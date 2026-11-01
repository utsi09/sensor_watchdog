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

static SensorConfig graph_only_cfg()
{
    SensorConfig c;
    c.id = "lidar";
    c.diag_checked = false;
    return c;
}

static SensorConfig diag_cfg(int error_level = 2)
{
    SensorConfig c;
    c.id = "gps";
    c.diag_checked = true;
    c.diag_timeout_sec = 3.0;
    c.diag_error_level = error_level;
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

static void test_graph_only_healthy()
{
    // 예측: diag 를 안 보는 센서는 노드·발행자만 있으면 NONE. 보고가 없어도 STALE 이 아니다.
    Health h(graph_only_cfg());
    h.reset(0.0);
    GraphView g;              // node 미검사, 발행자 1
    CHECK(h.diagnose(0.1, g) == Fault::NONE);
    CHECK(h.diagnose(100.0, g) == Fault::NONE);
    CHECK(h.diagnose_all(100.0, g).empty());
}

static void test_graph_faults()
{
    // 예측: node_name 을 검사하는데 그래프에 없으면 NODE_NOT_VISIBLE
    Health h(graph_only_cfg());
    h.reset(0.0);
    GraphView g;
    g.node_checked = true;
    g.node_count = 0;
    CHECK(!g.node_visible());
    CHECK(h.diagnose(2.0, g) == Fault::NODE_NOT_VISIBLE);

    // 예측: 노드는 있는데 발행자가 0 이면 NO_PUBLISHER
    g.node_count = 1;
    g.publishers = 0;
    CHECK(h.diagnose(2.0, g) == Fault::NO_PUBLISHER);

    // 예측: 둘 다 있으면 NONE
    g.publishers = 1;
    CHECK(h.diagnose(2.0, g) == Fault::NONE);

    // 예측: 노드 미검사(node_name 없음)면 node_count 는 무시, 발행자 0 → NO_PUBLISHER
    g.node_checked = false;
    g.node_count = 0;
    g.publishers = 0;
    CHECK(g.node_visible());
    CHECK(h.diagnose(2.0, g) == Fault::NO_PUBLISHER);
}

static void test_duplicate()
{
    // 예측: 발행자가 max_publishers(1) 보다 많으면 DUPLICATE
    Health h(graph_only_cfg());
    h.reset(0.0);
    GraphView g;
    g.publishers = 2;
    CHECK(h.diagnose(1.0, g) == Fault::DUPLICATE);

    // 예측: 같은 이름 노드가 둘이어도 DUPLICATE (발행자 1 이어도)
    g.publishers = 1;
    g.node_checked = true;
    g.node_count = 2;
    CHECK(h.diagnose(1.0, g) == Fault::DUPLICATE);

    // 예측: 노드 미검사면 node_count 2 는 의미 없음 → NONE
    g.node_checked = false;
    CHECK(h.diagnose(1.0, g) == Fault::NONE);

    // 예측: max_publishers 2 인 센서는 발행자 2 가 정상
    SensorConfig c = graph_only_cfg();
    c.max_publishers = 2;
    Health h2(c);
    h2.reset(0.0);
    GraphView g2;
    g2.publishers = 2;
    CHECK(h2.diagnose(1.0, g2) == Fault::NONE);
    g2.publishers = 3;
    CHECK(h2.diagnose(1.0, g2) == Fault::DUPLICATE);

    // 예측: 발행자 0 이 중복보다 먼저다 (노드 둘인데 발행자 없음 → NO_PUBLISHER)
    GraphView g3;
    g3.node_checked = true;
    g3.node_count = 2;
    g3.publishers = 0;
    CHECK(h.diagnose(1.0, g3) == Fault::NO_PUBLISHER);
}

static void test_duplicate_with_diag()
{
    // 예측: 중복 + diag ERROR 면 둘 다 보고하고, 중복이 앞에 온다
    Health h(diag_cfg());
    h.reset(0.0);
    h.on_diag(1.0, 2, "error");
    GraphView g;
    g.publishers = 2;
    const auto all = h.diagnose_all(1.5, g);
    CHECK(all.size() == 2);
    CHECK(all[0] == Fault::DUPLICATE && all[1] == Fault::DIAG_ERROR);
    CHECK(join_faults(all) == "DUPLICATE+DIAG_ERROR");
    CHECK(has_fault(all, Fault::DUPLICATE));
    CHECK(!has_fault(all, Fault::DIAG_STALE));
}

static void test_graph_before_diag()
{
    // 예측: 발행자가 없으면 diag 가 ERROR 여도 NO_PUBLISHER 하나만 보고한다
    Health h(diag_cfg());
    h.reset(0.0);
    h.on_diag(1.0, 2, "driver says error");
    GraphView g;
    g.publishers = 0;
    const auto all = h.diagnose_all(1.5, g);
    CHECK(all.size() == 1 && all.front() == Fault::NO_PUBLISHER);
}

static void test_diag_stale()
{
    // 예측: reset 직후 3초 안에는 보고가 없어도 NONE, 3초 넘으면 DIAG_STALE
    Health h(diag_cfg());
    h.reset(0.0);
    GraphView g;
    CHECK(h.diagnose(2.9, g) == Fault::NONE);
    CHECK(h.diagnose(3.1, g) == Fault::DIAG_STALE);
    CHECK(!h.received());

    // 예측: 보고가 들어오면 다시 NONE, 그 뒤 3초 지나면 다시 STALE
    h.on_diag(4.0, 0, "ok");
    CHECK(h.received());
    CHECK(h.diagnose(6.9, g) == Fault::NONE);
    CHECK(h.diagnose(7.1, g) == Fault::DIAG_STALE);
    CHECK(h.diag_age(7.1) > 3.0);
}

static void test_diag_error()
{
    // 예측: level 2(ERROR) 보고면 DIAG_ERROR, level 1(WARN) 은 기본 설정에서 NONE
    Health h(diag_cfg());
    h.reset(0.0);
    GraphView g;
    h.on_diag(1.0, 2, "no events in window");
    CHECK(h.diagnose(1.5, g) == Fault::DIAG_ERROR);
    CHECK(h.diag_level() == 2);
    CHECK(h.diag_message() == "no events in window");

    h.on_diag(2.0, 1, "slow");
    CHECK(h.diagnose(2.5, g) == Fault::NONE);

    h.on_diag(3.0, 0, "ok");
    CHECK(h.diagnose(3.5, g) == Fault::NONE);

    // 예측: level 3(STALE, diagnostic_aggregator 가 붙이는 값) 도 ERROR 이상이다
    h.on_diag(4.0, 3, "stale");
    CHECK(h.diagnose(4.5, g) == Fault::DIAG_ERROR);
}

static void test_diag_error_level_warn()
{
    // 예측: diag_error_level 1 이면 WARN 부터 DIAG_ERROR
    Health h(diag_cfg(1));
    h.reset(0.0);
    GraphView g;
    h.on_diag(1.0, 1, "warn");
    CHECK(h.diagnose(1.5, g) == Fault::DIAG_ERROR);
    h.on_diag(2.0, 0, "ok");
    CHECK(h.diagnose(2.5, g) == Fault::NONE);
}

static void test_diag_stale_over_error()
{
    // 예측: 마지막 보고가 ERROR 였어도 보고 자체가 끊기면 STALE 로 보고한다 (더 심한 상태)
    Health h(diag_cfg());
    h.reset(0.0);
    GraphView g;
    h.on_diag(1.0, 2, "error");
    CHECK(h.diagnose(2.0, g) == Fault::DIAG_ERROR);
    CHECK(h.diagnose(4.5, g) == Fault::DIAG_STALE);
}

static void test_reset_clears_diag()
{
    // 예측: 재시작 뒤 reset 하면 이전 ERROR 는 잊고, 보고가 올 때까지 3초 유예
    Health h(diag_cfg());
    h.reset(0.0);
    GraphView g;
    h.on_diag(1.0, 2, "error");
    CHECK(h.diagnose(1.5, g) == Fault::DIAG_ERROR);
    h.reset(10.0);
    CHECK(h.diagnose(10.5, g) == Fault::NONE);
    CHECK(h.diag_level() == 0);
    CHECK(h.diagnose(13.5, g) == Fault::DIAG_STALE);
}

static void test_join_faults()
{
    CHECK(join_faults({}) == "NONE");
    CHECK(join_faults({Fault::NO_PUBLISHER}) == "NO_PUBLISHER");
    CHECK(join_faults({Fault::DIAG_STALE, Fault::DIAG_ERROR}) == "DIAG_STALE+DIAG_ERROR");
    // 예측: 모든 고장 이름이 to_string 으로 나오고 LAST_FAULT 까지 빠짐없다
    for (int i = 0; i <= static_cast<int>(LAST_FAULT); ++i)
        CHECK(std::string(to_string(static_cast<Fault>(i))) != "?");
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
    CHECK(l.evaluate(5.0, Fault::NO_PUBLISHER) == Verdict::GRACE);

    // 예측: 10초 지나 고장 시작 → HOLDING, 1초 뒤 ACT (service 칸)
    CHECK(l.evaluate(10.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(10.5, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(11.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);
    CHECK(l.bad_since() == 10.0);

    // 행동 시작 → 완료(3초 걸림)
    l.on_action_started(11.0);
    l.on_action_finished(14.0);
    CHECK(l.attempts() == 1);
    CHECK(l.total_actions() == 1);

    // 예측: 완료 후 10초 유예, 그 뒤 고장이면 cooldown(완료+30초)까지 COOLDOWN
    CHECK(l.evaluate(20.0, Fault::NO_PUBLISHER) == Verdict::GRACE);
    CHECK(l.evaluate(24.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(26.0, Fault::NO_PUBLISHER) == Verdict::COOLDOWN);
    CHECK(l.evaluate(44.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);  // 아직 같은 칸 (2회까지)

    // 두 번째 service 시도 완료 → 다음 칸(power)으로
    l.on_action_started(44.0);
    l.on_action_finished(47.0);
    CHECK(l.current() == Step::POWER);
    CHECK(l.attempts() == 0);

    // power 2회 → EXHAUSTED
    CHECK(l.evaluate(80.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(81.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    l.on_action_started(81.0);
    l.on_action_finished(100.0);
    CHECK(l.evaluate(140.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(141.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    l.on_action_started(141.0);
    l.on_action_finished(160.0);
    CHECK(l.exhausted());
    CHECK(l.evaluate(200.0, Fault::NO_PUBLISHER) == Verdict::EXHAUSTED);
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
    CHECK(l.evaluate(11.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(12.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    l.on_action_started(12.0);
    l.on_action_finished(15.0);
    CHECK(l.evaluate(30.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.evaluate(100.0, Fault::NONE) == Verdict::HEALTHY);
    CHECK(l.evaluate(200.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(201.0, Fault::NO_PUBLISHER) == Verdict::ACT);
    CHECK(l.current() == Step::SERVICE);
    CHECK(l.attempts() == 1);
}

static void test_ladder_per_fault_hold()
{
    // 예측: 같은 Ladder 에서 hold 를 10초로 주면 10초 전엔 HOLDING, 지나면 ACT
    Ladder l(policy_two_steps());
    l.reset(0.0);
    CHECK(l.evaluate(20.0, Fault::DIAG_ERROR, 10.0) == Verdict::HOLDING);
    CHECK(l.evaluate(29.9, Fault::DIAG_ERROR, 10.0) == Verdict::HOLDING);
    CHECK(l.evaluate(30.0, Fault::DIAG_ERROR, 10.0) == Verdict::ACT);
}

static void test_ladder_empty()
{
    // 예측: 사다리 없음 = 감시만. 고장이면 바로 EXHAUSTED
    Policy p = policy_two_steps();
    p.ladder.clear();
    Ladder l(p);
    l.reset(0.0);
    CHECK(l.evaluate(20.0, Fault::NO_PUBLISHER) == Verdict::EXHAUSTED);
    CHECK(l.evaluate(21.0, Fault::NONE) == Verdict::HEALTHY);
}

static void test_ladder_defer()
{
    Policy p;
    p.grace_sec = 10.0;
    p.hold_sec = 2.0;
    p.cooldown_sec = 30.0;
    p.ladder = {Step::SERVICE};
    Ladder l(p);
    l.reset(0.0);

    // 예측: 15초부터 고장 → 17초에 ACT
    CHECK(l.evaluate(15.0, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(17.0, Fault::NO_PUBLISHER) == Verdict::ACT);

    // 예측: 형제 센서가 20초에 유닛을 재시작 → 이 센서는 유예(30초까지), 시도 횟수 0 유지
    l.defer(20.0);
    CHECK(l.evaluate(25.0, Fault::NO_PUBLISHER) == Verdict::GRACE);
    CHECK(l.attempts() == 0);
    CHECK(l.bad_since() < 0.0);

    // 예측: 유예 뒤에도 고장이면 hold 를 다시 세고(32초), 그 뒤엔 쿨다운(50초까지)에 걸린다
    CHECK(l.evaluate(30.5, Fault::NO_PUBLISHER) == Verdict::HOLDING);
    CHECK(l.evaluate(33.0, Fault::NO_PUBLISHER) == Verdict::COOLDOWN);
    CHECK(l.evaluate(50.0, Fault::NO_PUBLISHER) == Verdict::ACT);
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

static void test_common_cause_by_domain()
{
    // 예측: 라이다 4대가 같은 유닛(lidar)에서 동시에 죽어도 복구 단위는 1개 → 공통 원인 아님
    CHECK(!common_cause_by_domain(
        {{"lidar", 10.0}, {"lidar", 10.1}, {"lidar", 10.2}, {"lidar", 10.3}, {"gps", -1.0}},
        2, 3.0));
    // 예측: 라이다 유닛과 GPS 유닛이 3초 안에 같이 죽으면 공통 원인
    CHECK(common_cause_by_domain(
        {{"lidar", 10.0}, {"lidar", 10.1}, {"lidar", -1.0}, {"lidar", -1.0}, {"gps", 12.0}},
        2, 3.0));
    // 예측: 유닛별로 가장 최근 고장 시작을 쓴다. 앞 라이다는 10.0 부터 죽어 있었고,
    //       나중에 호스트가 죽어 나머지 라이다(40.0)와 GPS(40.2)가 같이 끊김 → 공통 원인
    CHECK(common_cause_by_domain(
        {{"lidar", 10.0}, {"lidar", 40.0}, {"lidar", 40.0}, {"gps", 40.2}},
        2, 3.0));
    // 예측: 라이다 유닛은 10.0, GPS 는 14.0 에 따로 죽음 → 간격 4초 → 아님
    CHECK(!common_cause_by_domain(
        {{"lidar", 10.0}, {"lidar", 10.0}, {"gps", 14.0}},
        2, 3.0));
    // 예측: min_count 가 복구 단위 수보다 크면 절대 true 가 아니다
    CHECK(!common_cause_by_domain({{"lidar", 10.0}, {"gps", 10.0}}, 3, 3.0));
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

    // 예측: autoware_vehicle_msgs/GearReport 는 stamp(8바이트) 뒤 바로 uint8 report.
    //       Header 가 아니므로 Prefix::STAMP 로 읽어야 22(PARK) 가 나온다.
    const uint8_t gr[] = {0x00, 0x01, 0x00, 0x00,
                          0x10, 0x00, 0x00, 0x00,   // sec
                          0x20, 0x00, 0x00, 0x00,   // nanosec
                          22,                       // report = PARK
                          0x00, 0x00, 0x00};        // 패딩
    auto park = read_first_int(gr, sizeof(gr), Prefix::STAMP, 1);
    CHECK(park.has_value() && *park == 22);
    // 예측: 같은 바이트를 HEADER 로 잘못 읽으면 22 를 frame_id 길이로 보고 길이 부족 → nullopt
    CHECK(!read_first_int(gr, sizeof(gr), Prefix::HEADER, 1).has_value());
    // 예측: NONE 으로 읽으면 sec 의 첫 바이트(0x10) 가 나온다 (설정 실수 시 증상)
    auto wrong = read_first_int(gr, sizeof(gr), Prefix::NONE, 1);
    CHECK(wrong.has_value() && *wrong == 0x10);

    // 예측: stamp 뒤 int32 필드는 8바이트 오프셋이 이미 4의 배수라 패딩 없음
    const uint8_t gr4[] = {0x00, 0x01, 0x00, 0x00,
                           0, 0, 0, 0,  0, 0, 0, 0,
                           0x07, 0x00, 0x00, 0x00};
    auto v32 = read_first_int(gr4, sizeof(gr4), Prefix::STAMP, 4);
    CHECK(v32.has_value() && *v32 == 7);

    // 예측: 문자열 파싱
    Prefix pf;
    CHECK(parse_prefix("none", pf) && pf == Prefix::NONE);
    CHECK(parse_prefix("stamp", pf) && pf == Prefix::STAMP);
    CHECK(parse_prefix("header", pf) && pf == Prefix::HEADER);
    CHECK(!parse_prefix("Header", pf));
}

int main()
{
    test_cdr();
    test_graph_only_healthy();
    test_graph_faults();
    test_duplicate();
    test_duplicate_with_diag();
    test_graph_before_diag();
    test_diag_stale();
    test_diag_error();
    test_diag_error_level_warn();
    test_diag_stale_over_error();
    test_reset_clears_diag();
    test_join_faults();
    test_ladder_flow();
    test_ladder_recovers_between();
    test_ladder_empty();
    test_ladder_per_fault_hold();
    test_ladder_defer();
    test_common_cause();
    test_common_cause_by_domain();
    test_parse_step();
    test_cdr_first_int();

    if (failures == 0) {
        std::printf("ALL PASSED\n");
        return 0;
    }
    std::printf("%d FAILED\n", failures);
    return 1;
}
