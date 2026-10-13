// ROS 없이 돌아가는 판정 로직 테스트.
// 각 테스트는 "이렇게 되어야 한다"를 먼저 적고 확인한다.
//
// 단독 빌드: g++ -std=c++17 -Wall -Wextra -I include test/test_health.cpp -o test_health

#include "sensor_watchdog/cdr_header.hpp"
#include "sensor_watchdog/health.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace sensor_watchdog;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);    \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

// 10 Hz 센서를 흉내 내는 보조 함수. from~to 초 동안 메시지를 넣는다.
static void feed(Health &h, double from, double to, double period,
                 double stamp_start = 100.0, bool advance = true, std::size_t bytes = 1000)
{
    for (double t = from; t < to; t += period) {
        const double stamp = advance ? stamp_start + (t - from) : stamp_start;
        const int32_t sec = static_cast<int32_t>(stamp);
        const uint32_t nsec = static_cast<uint32_t>((stamp - sec) * 1e9);
        h.on_message(t, /*ros_now=*/t + 1000.0, sec, nsec, bytes);
    }
}

static SensorConfig lidar_cfg()
{
    SensorConfig c;
    c.id = "lidar";
    c.timeout_sec = 1.0;
    c.stamp_timeout_sec = 1.0;
    c.min_hz = 5.0;
    c.hz_window_sec = 5.0;
    c.min_bytes = 100;
    return c;
}

static void test_cdr()
{
    // 예측: LE 인캡슐레이션 + sec=0x12345678, nsec=0x9ABCDEF0 이 그대로 읽힌다.
    const uint8_t le[] = {0x00, 0x01, 0x00, 0x00,
                          0x78, 0x56, 0x34, 0x12,
                          0xF0, 0xDE, 0xBC, 0x9A,
                          0x00};
    auto s = read_header_stamp(le, sizeof(le));
    CHECK(s.has_value());
    CHECK(s->sec == 0x12345678);
    CHECK(s->nanosec == 0x9ABCDEF0u);

    // 예측: BE도 같은 값
    const uint8_t be[] = {0x00, 0x00, 0x00, 0x00,
                          0x12, 0x34, 0x56, 0x78,
                          0x9A, 0xBC, 0xDE, 0xF0};
    auto b = read_header_stamp(be, sizeof(be));
    CHECK(b.has_value());
    CHECK(b->sec == 0x12345678);
    CHECK(b->nanosec == 0x9ABCDEF0u);

    // 예측: 12바이트 미만, 잘못된 인캡슐레이션은 nullopt
    CHECK(!read_header_stamp(le, 11).has_value());
    const uint8_t bad[] = {0x00, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(!read_header_stamp(bad, sizeof(bad)).has_value());
}

static void test_healthy()
{
    // 예측: 10 Hz로 6초 받으면 NONE, hz는 약 10
    Health h(lidar_cfg());
    h.reset(0.0);
    feed(h, 0.0, 6.0, 0.1);
    GraphView g;
    CHECK(h.diagnose(6.0, g) == Fault::NONE);
    CHECK(h.hz(6.0) > 9.0 && h.hz(6.0) <= 10.5);
}

static void test_rx_timeout()
{
    // 예측: 6초까지 받고 7.5초에 보면 RX_TIMEOUT
    Health h(lidar_cfg());
    h.reset(0.0);
    feed(h, 0.0, 6.0, 0.1);
    GraphView g;
    CHECK(h.diagnose(7.5, g) == Fault::RX_TIMEOUT);

    // 예측: 아무것도 안 받았으면 RX_TIMEOUT
    Health h2(lidar_cfg());
    h2.reset(0.0);
    CHECK(h2.diagnose(2.0, g) == Fault::RX_TIMEOUT);
}

static void test_graph_first()
{
    // 예측: 그래프 문제는 수신 문제보다 먼저 보고된다
    Health h(lidar_cfg());
    h.reset(0.0);
    GraphView g;
    g.node_checked = true;
    g.node_visible = false;
    CHECK(h.diagnose(2.0, g) == Fault::NODE_NOT_VISIBLE);

    g.node_visible = true;
    g.publishers = 0;
    CHECK(h.diagnose(2.0, g) == Fault::NO_PUBLISHER);

    g.publishers = 1;
    g.matched = 0;
    CHECK(h.diagnose(2.0, g) == Fault::NO_MATCHED_PUBLISHER);
}

static void test_stamp_not_advancing()
{
    // 예측: 메시지는 10 Hz로 오는데 stamp가 같으면 1초 뒤 STAMP_NOT_ADVANCING
    Health h(lidar_cfg());
    h.reset(0.0);
    feed(h, 0.0, 6.0, 0.1, 100.0, /*advance=*/false);
    GraphView g;
    CHECK(h.diagnose(6.0, g) == Fault::STAMP_NOT_ADVANCING);
}

static void test_stamp_backward_and_invalid()
{
    // 예측: stamp가 뒤로 가면 STAMP_INVALID
    Health h(lidar_cfg());
    h.reset(0.0);
    h.on_message(0.1, 1000.0, 100, 0, 1000);
    h.on_message(0.2, 1000.0, 99, 0, 1000);
    GraphView g;
    CHECK(h.diagnose(0.3, g) == Fault::STAMP_INVALID);

    // 예측: 파싱 실패(sec=-1)도 STAMP_INVALID
    Health h2(lidar_cfg());
    h2.reset(0.0);
    h2.on_message(0.1, 1000.0, -1, 0, 1000);
    CHECK(h2.diagnose(0.2, g) == Fault::STAMP_INVALID);

    // 예측: stamp 검사를 둘 다 끄면 무시된다
    SensorConfig c = lidar_cfg();
    c.check_stamp_progress = false;
    c.check_stamp_age = false;
    Health h3(c);
    h3.reset(0.0);
    h3.on_message(0.1, 1000.0, -1, 0, 1000);
    CHECK(h3.diagnose(0.2, g) == Fault::NONE);
}

static void test_too_small()
{
    // 예측: min_bytes 미만이면 TOO_SMALL
    Health h(lidar_cfg());
    h.reset(0.0);
    feed(h, 0.0, 1.0, 0.1, 100.0, true, /*bytes=*/50);
    GraphView g;
    CHECK(h.diagnose(1.0, g) == Fault::TOO_SMALL);
}

static void test_low_hz()
{
    // 예측: 2 Hz로 오면 hz_window(5초)가 찬 뒤 LOW_HZ. 그 전에는 판정 안 함.
    Health h(lidar_cfg());
    h.reset(0.0);
    feed(h, 0.0, 3.0, 0.5);
    GraphView g;
    CHECK(h.diagnose(3.0, g) == Fault::NONE);   // 아직 5초 안 참

    feed(h, 3.0, 6.0, 0.5);
    CHECK(h.diagnose(6.0, g) == Fault::LOW_HZ);
}

static void test_age()
{
    // 예측: check_stamp_age=true 이고 메시지가 2초 늦으면 STAMP_AGE
    SensorConfig c = lidar_cfg();
    c.check_stamp_age = true;
    c.max_age_sec = 0.5;
    Health h(c);
    h.reset(0.0);
    h.on_message(0.1, /*ros_now=*/102.0, 100, 0, 1000);
    GraphView g;
    CHECK(h.diagnose(0.2, g) == Fault::STAMP_AGE);
}

static Policy policy_two_steps()
{
    Policy p;
    p.grace_sec = 10.0;
    p.hold_sec = 1.0;
    p.cooldown_sec = 30.0;
    p.attempts_per_step = 2;
    p.healthy_reset_sec = 600.0;
    p.ladder = {Step::SERVICE, Step::POWER};
    return p;
}

static void test_ladder_flow()
{
    Ladder l(policy_two_steps());
    l.reset(0.0);

    // 예측: 유예 10초 동안은 고장이어도 GRACE
    CHECK(l.evaluate(5.0, Fault::RX_TIMEOUT) == Verdict::GRACE);

    // 예측: 10초 지나 고장 시작 → HOLDING, 1초 뒤 ACT (service 칸)
    CHECK(l.evaluate(10.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(10.5, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(11.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);
    CHECK(l.bad_since() == 10.0);

    // 행동 시작 → 완료(3초 걸림)
    l.on_action_started(11.0);
    l.on_action_finished(14.0);
    CHECK(l.attempts() == 1);
    CHECK(l.total_actions() == 1);

    // 예측: 완료 후 10초 유예, 그 뒤 고장이면 cooldown(완료+30초)까지 COOLDOWN
    CHECK(l.evaluate(20.0, Fault::RX_TIMEOUT) == Verdict::GRACE);
    CHECK(l.evaluate(24.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(26.0, Fault::RX_TIMEOUT) == Verdict::COOLDOWN);
    CHECK(l.evaluate(44.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);  // 아직 같은 칸 (2회까지)

    // 두 번째 service 시도 완료 → 다음 칸(power)으로
    l.on_action_started(44.0);
    l.on_action_finished(47.0);
    CHECK(l.current() == Step::POWER);
    CHECK(l.attempts() == 0);

    // power 2회 → EXHAUSTED
    CHECK(l.evaluate(80.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(81.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    l.on_action_started(81.0);
    l.on_action_finished(100.0);
    CHECK(l.evaluate(140.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(141.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    l.on_action_started(141.0);
    l.on_action_finished(160.0);
    CHECK(l.exhausted());
    CHECK(l.evaluate(200.0, Fault::RX_TIMEOUT) == Verdict::EXHAUSTED);
    CHECK(l.total_actions() == 4);

    // 예측: 정상으로 돌아와 600초 유지되면 사다리 초기화
    CHECK(l.evaluate(300.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.evaluate(899.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.exhausted());
    CHECK(l.evaluate(900.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(!l.exhausted());
    CHECK(l.step_index() == 0);
}

static void test_ladder_recovers_between()
{
    // 예측: 행동 1회 후 정상 복귀 → 다시 고장 나도 같은 칸 유지(2회째), 이후 다음 칸
    Ladder l(policy_two_steps());
    l.reset(0.0);
    CHECK(l.evaluate(11.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(12.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    l.on_action_started(12.0);
    l.on_action_finished(15.0);
    CHECK(l.evaluate(30.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.evaluate(100.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.evaluate(200.0, Fault::RX_TIMEOUT) == Verdict::HOLDING);
    CHECK(l.evaluate(201.0, Fault::RX_TIMEOUT) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);
    CHECK(l.attempts() == 1);
}

static void test_ladder_empty()
{
    // 예측: 사다리 없음 = 감시만. 고장이면 바로 EXHAUSTED
    Policy p = policy_two_steps();
    p.ladder.clear();
    Ladder l(p);
    l.reset(0.0);
    CHECK(l.evaluate(20.0, Fault::RX_TIMEOUT) == Verdict::EXHAUSTED);
    CHECK(l.evaluate(21.0, Fault::NONE) == Verdict::HEALTHY);
}

static void test_common_cause()
{
    // 예측: 3개 중 2개가 3초 안에 같이 죽으면 true
    CHECK(common_cause({10.0, 11.5, -1.0}, 2, 3.0));
    // 예측: 간격이 window보다 크면 false
    CHECK(!common_cause({10.0, 20.0, -1.0}, 2, 3.0));
    // 예측: 하나만 죽으면 false
    CHECK(!common_cause({10.0, -1.0, -1.0}, 2, 3.0));
    // 예측: min_count=3 이면 세 개 모두 고장이어야 true
    CHECK(!common_cause({10.0, 11.0, -1.0}, 3, 3.0));
    CHECK(common_cause({10.0, 11.0, 12.9}, 3, 3.0));
    // 예측: min_count<2 는 항상 false (설정 실수 방지)
    CHECK(!common_cause({10.0, 10.0, 10.0}, 1, 3.0));
}

static void test_parse_step()
{
    Step s;
    CHECK(parse_step("lifecycle", s) && s == Step::LIFECYCLE);
    CHECK(parse_step("service", s) && s == Step::SERVICE);
    CHECK(parse_step("power", s) && s == Step::POWER);
    CHECK(!parse_step("reboot", s));
}

static void test_cdr_first_int()
{
    // 예측: std_msgs/Bool (헤더 없음, 1바이트) true → 1
    const uint8_t b[] = {0x00, 0x01, 0x00, 0x00, 0x01};
    auto v = read_first_int(b, sizeof(b), false, 1);
    CHECK(v.has_value() && *v == 1);

    // 예측: 헤더(frame_id "") 뒤 uint8 기어=22 → 22. 길이 1(널) 뒤 정렬 없음(1바이트)
    const uint8_t g[] = {0x00, 0x01, 0x00, 0x00,
                         0x10, 0x00, 0x00, 0x00,   // sec
                         0x20, 0x00, 0x00, 0x00,   // nanosec
                         0x01, 0x00, 0x00, 0x00,   // frame_id len=1
                         0x00,                     // "\0"
                         22};
    auto gear = read_first_int(g, sizeof(g), true, 1);
    CHECK(gear.has_value() && *gear == 22);

    // 예측: 헤더 뒤 int32 필드면 4바이트 정렬을 건너뛴다 (13 → 16)
    const uint8_t g4[] = {0x00, 0x01, 0x00, 0x00,
                          0, 0, 0, 0,  0, 0, 0, 0,
                          0x01, 0, 0, 0,  0x00,
                          0xAA, 0xBB, 0xCC,         // 패딩
                          0x05, 0x00, 0x00, 0x00};
    auto v4 = read_first_int(g4, sizeof(g4), true, 4);
    CHECK(v4.has_value() && *v4 == 5);

    // 예측: 길이가 모자라면 nullopt
    CHECK(!read_first_int(g4, sizeof(g4) - 1, true, 4).has_value());
}

int main()
{
    test_cdr();
    test_healthy();
    test_rx_timeout();
    test_graph_first();
    test_stamp_not_advancing();
    test_stamp_backward_and_invalid();
    test_too_small();
    test_low_hz();
    test_age();
    test_ladder_flow();
    test_ladder_recovers_between();
    test_ladder_empty();
    test_common_cause();
    test_parse_step();
    test_cdr_first_int();

    if (failures == 0) {
        std::printf("ALL PASSED\n");
        return 0;
    }
    std::printf("%d FAILED\n", failures);
    return 1;
}
