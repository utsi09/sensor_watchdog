#pragma once
// ROS에 의존하지 않는 순수 판정 로직.
// 1) Health        : 관측(수신 시각, stamp, 크기) → 고장 코드
// 2) Ladder        : 고장 코드 + 시간 → 지금 뭘 할 차례인지(유예/대기/행동/포기)
// 3) common_cause  : 여러 센서가 동시에 죽었는지
//
// 이 파일은 test/test_health.cpp 로 ROS 없이 단독 테스트된다.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace sensor_watchdog
{

// ---------------------------------------------------------------- 설정

struct SensorConfig
{
    std::string id;

    double timeout_sec = 1.0;         // 마지막 수신 후 허용 시간
    double stamp_timeout_sec = 1.0;   // stamp가 전진하지 않는 상태 허용 시간
    double min_hz = 0.0;              // 0이면 Hz 검사 안 함
    double hz_window_sec = 5.0;
    int64_t min_bytes = 1;            // 직렬화 크기 하한 (원시 로그 보고 정한다)

    bool check_stamp_progress = true;
    bool check_stamp_age = false;     // PTP로 시각 맞춘 뒤에 켠다
    double max_age_sec = 0.5;
};

// ---------------------------------------------------------------- 고장 코드

enum class Fault
{
    NONE,
    NODE_NOT_VISIBLE,
    NO_PUBLISHER,
    NO_MATCHED_PUBLISHER,
    RX_TIMEOUT,
    TOO_SMALL,
    STAMP_INVALID,
    STAMP_AGE,
    STAMP_NOT_ADVANCING,
    LOW_HZ
};

inline const char *to_string(Fault f)
{
    switch (f) {
    case Fault::NONE:                 return "NONE";
    case Fault::NODE_NOT_VISIBLE:     return "NODE_NOT_VISIBLE";
    case Fault::NO_PUBLISHER:         return "NO_PUBLISHER";
    case Fault::NO_MATCHED_PUBLISHER: return "NO_MATCHED_PUBLISHER_CHECK_QOS_TYPE";
    case Fault::RX_TIMEOUT:           return "RX_TIMEOUT";
    case Fault::TOO_SMALL:            return "TOO_SMALL";
    case Fault::STAMP_INVALID:        return "STAMP_INVALID_OR_BACKWARD";
    case Fault::STAMP_AGE:            return "STAMP_AGE_OR_CLOCK";
    case Fault::STAMP_NOT_ADVANCING:  return "STAMP_NOT_ADVANCING";
    case Fault::LOW_HZ:               return "LOW_RX_HZ";
    }
    return "?";
}

// ROS 그래프에서 본 정보. 노드 쪽에서 채워서 넘긴다.
struct GraphView
{
    bool node_checked = false;   // node_name이 설정돼 있는가
    bool node_visible = true;
    std::size_t publishers = 1;  // count_publishers(topic)
    std::size_t matched = 1;     // subscription->get_publisher_count()
};

// ---------------------------------------------------------------- Health

class Health
{
public:
    explicit Health(SensorConfig cfg) : cfg_(std::move(cfg)) {}

    const SensorConfig &config() const { return cfg_; }

    // 재시작 직후 등, 관측을 처음부터 다시 쌓을 때
    void reset(double now)
    {
        reset_at_ = now;
        last_rx_ = now;
        last_advance_ = now;
        received_ = false;
        last_stamp_ = 0;
        stamp_bad_ = false;
        too_small_ = false;
        age_ = 0.0;
        arrivals_.clear();
    }

    // now     : steady clock 초 (수신 간격 측정용)
    // ros_now : ROS 시각 초 (stamp 나이 계산용, check_stamp_age일 때만 의미)
    void on_message(double now, double ros_now, int32_t sec, uint32_t nsec, std::size_t bytes)
    {
        const int64_t stamp = int64_t(sec) * 1000000000LL + int64_t(nsec);

        stamp_bad_ =
            sec < 0 ||
            nsec >= 1000000000U ||
            stamp <= 0 ||
            (received_ && stamp < last_stamp_);

        if (!received_ || stamp > last_stamp_)
            last_advance_ = now;

        last_stamp_ = stamp;
        last_rx_ = now;
        received_ = true;
        ++count_;

        too_small_ = static_cast<int64_t>(bytes) < cfg_.min_bytes;
        age_ = ros_now - static_cast<double>(stamp) * 1e-9;

        arrivals_.push_back(now);
        trim(now);
    }

    Fault diagnose(double now, const GraphView &g)
    {
        trim(now);

        if (g.node_checked && !g.node_visible)
            return Fault::NODE_NOT_VISIBLE;

        if (g.publishers == 0)
            return Fault::NO_PUBLISHER;

        if (g.matched == 0)
            return Fault::NO_MATCHED_PUBLISHER;

        if (!received_ || now - last_rx_ > cfg_.timeout_sec)
            return Fault::RX_TIMEOUT;

        if (too_small_)
            return Fault::TOO_SMALL;

        if ((cfg_.check_stamp_progress || cfg_.check_stamp_age) && stamp_bad_)
            return Fault::STAMP_INVALID;

        if (cfg_.check_stamp_age && (age_ > cfg_.max_age_sec || age_ < -0.1))
            return Fault::STAMP_AGE;

        if (cfg_.check_stamp_progress && now - last_advance_ > cfg_.stamp_timeout_sec)
            return Fault::STAMP_NOT_ADVANCING;

        if (cfg_.min_hz > 0.0 &&
            now - reset_at_ >= cfg_.hz_window_sec &&
            hz(now) < cfg_.min_hz)
        {
            return Fault::LOW_HZ;
        }

        return Fault::NONE;
    }

    double hz(double now)
    {
        trim(now);
        return static_cast<double>(arrivals_.size()) / cfg_.hz_window_sec;
    }

    bool received() const { return received_; }
    double age() const { return age_; }
    uint64_t count() const { return count_; }
    int64_t last_stamp() const { return last_stamp_; }

private:
    void trim(double now)
    {
        while (!arrivals_.empty() && now - arrivals_.front() > cfg_.hz_window_sec)
            arrivals_.pop_front();
    }

    SensorConfig cfg_;

    double reset_at_ = 0.0;
    double last_rx_ = 0.0;
    double last_advance_ = 0.0;
    double age_ = 0.0;

    bool received_ = false;
    bool stamp_bad_ = false;
    bool too_small_ = false;

    int64_t last_stamp_ = 0;
    uint64_t count_ = 0;

    std::deque<double> arrivals_;
};

// ---------------------------------------------------------------- Ladder

// 복구 사다리의 한 칸. 가벼운 것부터.
enum class Step
{
    LIFECYCLE,   // 프로세스 유지, lifecycle deactivate → activate (ms 단위)
    SERVICE,     // systemctl restart (초 단위)
    POWER        // 전원 리셋 스크립트 (수십 초)
};

inline const char *to_string(Step s)
{
    switch (s) {
    case Step::LIFECYCLE: return "lifecycle";
    case Step::SERVICE:   return "service";
    case Step::POWER:     return "power";
    }
    return "?";
}

inline bool parse_step(const std::string &name, Step &out)
{
    if (name == "lifecycle") { out = Step::LIFECYCLE; return true; }
    if (name == "service")   { out = Step::SERVICE;   return true; }
    if (name == "power")     { out = Step::POWER;     return true; }
    return false;
}

struct Policy
{
    double grace_sec = 10.0;          // 시작/행동 직후 판정 유예
    double hold_sec = 1.0;            // 고장이 이만큼 지속돼야 행동
    double cooldown_sec = 30.0;       // 행동 완료 후 다음 행동까지 최소 간격
    int attempts_per_step = 2;        // 같은 칸을 몇 번 시도하고 다음 칸으로
    double healthy_reset_sec = 600.0; // 이만큼 정상이면 사다리를 처음으로
    std::vector<Step> ladder;         // 비어 있으면 감시만
};

enum class Verdict
{
    GRACE,      // 유예 중
    HEALTHY,
    HOLDING,    // 고장이지만 아직 hold_sec 안 지남
    COOLDOWN,   // 고장이지만 직전 행동 후 대기 중
    ACT,        // 지금 current() 칸을 실행할 차례
    EXHAUSTED   // 사다리 끝. 사람 호출
};

inline const char *to_string(Verdict v)
{
    switch (v) {
    case Verdict::GRACE:     return "GRACE";
    case Verdict::HEALTHY:   return "HEALTHY";
    case Verdict::HOLDING:   return "HOLDING";
    case Verdict::COOLDOWN:  return "COOLDOWN";
    case Verdict::ACT:       return "ACT";
    case Verdict::EXHAUSTED: return "EXHAUSTED";
    }
    return "?";
}

class Ladder
{
public:
    explicit Ladder(Policy p) : policy_(std::move(p)) {}

    const Policy &policy() const { return policy_; }

    void reset(double now) { grace_until_ = now + policy_.grace_sec; }

    Verdict evaluate(double now, Fault fault)
    {
        return evaluate(now, fault, policy_.hold_sec);
    }

    // hold_sec: 이 고장 종류에 적용할 대기 시간. 고장마다 다르게 줄 수 있다.
    Verdict evaluate(double now, Fault fault, double hold_sec)
    {
        if (now < grace_until_)
            return Verdict::GRACE;

        if (fault == Fault::NONE) {
            bad_since_ = -1.0;

            if (healthy_since_ < 0.0)
                healthy_since_ = now;

            if (now - healthy_since_ >= policy_.healthy_reset_sec) {
                step_ = 0;
                attempts_ = 0;
                exhausted_ = false;
            }
            return Verdict::HEALTHY;
        }

        healthy_since_ = -1.0;

        if (bad_since_ < 0.0)
            bad_since_ = now;

        if (exhausted_ || policy_.ladder.empty())
            return Verdict::EXHAUSTED;

        if (now - bad_since_ < hold_sec)
            return Verdict::HOLDING;

        if (now < next_action_)
            return Verdict::COOLDOWN;

        return Verdict::ACT;
    }

    Step current() const { return policy_.ladder[step_]; }
    int attempts() const { return attempts_; }
    int total_actions() const { return total_; }
    std::size_t step_index() const { return step_; }
    bool exhausted() const { return exhausted_; }
    double bad_since() const { return bad_since_; }

    void on_action_started(double /*now*/)
    {
        ++attempts_;
        ++total_;
        bad_since_ = -1.0;
        healthy_since_ = -1.0;
    }

    // 같은 유닛의 다른 센서가 행동했을 때. 이 센서도 함께 끊기고 올라오므로
    // 유예와 쿨다운을 같이 받는다. 시도 횟수는 올리지 않는다.
    void defer(double now)
    {
        grace_until_ = now + policy_.grace_sec;
        next_action_ = now + policy_.cooldown_sec;
        bad_since_ = -1.0;
        healthy_since_ = -1.0;
    }

    void on_action_finished(double now)
    {
        grace_until_ = now + policy_.grace_sec;
        next_action_ = now + policy_.cooldown_sec;

        if (attempts_ >= policy_.attempts_per_step) {
            attempts_ = 0;
            if (step_ + 1 < policy_.ladder.size())
                ++step_;
            else
                exhausted_ = true;
        }
    }

private:
    Policy policy_;

    double grace_until_ = 0.0;
    double next_action_ = 0.0;
    double bad_since_ = -1.0;
    double healthy_since_ = -1.0;

    std::size_t step_ = 0;
    int attempts_ = 0;
    int total_ = 0;
    bool exhausted_ = false;
};

// ---------------------------------------------------------------- 공통 원인

// 고장 시작 시각 목록(정상이면 음수)에서,
// window 초 안에 min_count 개 이상이 함께 고장 났으면 true.
// 그 경우 센서 탓이 아니라 네트워크/호스트 탓일 가능성이 크므로 재시작하지 않는다.
inline bool common_cause(std::vector<double> bad_since, int min_count, double window)
{
    bad_since.erase(
        std::remove_if(bad_since.begin(), bad_since.end(),
                       [](double t) { return t < 0.0; }),
        bad_since.end());

    if (min_count < 2 || static_cast<int>(bad_since.size()) < min_count)
        return false;

    std::sort(bad_since.begin(), bad_since.end());

    const std::size_t k = static_cast<std::size_t>(min_count);
    for (std::size_t i = 0; i + k - 1 < bad_since.size(); ++i) {
        if (bad_since[i + k - 1] - bad_since[i] <= window)
            return true;
    }
    return false;
}


// 센서가 아니라 "복구 단위"(같은 systemd 유닛을 공유하는 센서 묶음) 기준의 공통 원인 판정.
// 라이다 4대가 한 런치(유닛)에서 돌면 드라이버가 죽을 때 4개가 동시에 끊긴다.
// 그건 공통 원인이 아니라 그 유닛 하나의 고장이므로, 유닛별로 고장 시각 하나만 세어
// common_cause 에 넘긴다. 유닛 안에서는 가장 최근에 시작된 고장을 쓴다: "지금 여러 유닛이
// 동시에 죽었나"를 묻는 것이므로, 한 센서의 오래된 고장이 유닛 전체의 시각을 과거로 묶어 두면
// 나중에 호스트가 죽어 전부 끊겨도 동시성이 보이지 않는다.
//   domains: (복구 단위 키, bad_since) 쌍. 정상 센서는 bad_since 음수.
inline bool common_cause_by_domain(
    const std::vector<std::pair<std::string, double>> &domains, int min_count, double window)
{
    std::vector<std::pair<std::string, double>> latest;
    for (const auto &d : domains) {
        if (d.second < 0.0)
            continue;
        auto it = std::find_if(latest.begin(), latest.end(),
                               [&](const auto &e) { return e.first == d.first; });
        if (it == latest.end())
            latest.emplace_back(d.first, d.second);
        else
            it->second = std::max(it->second, d.second);
    }

    std::vector<double> bad_since;
    for (const auto &e : latest)
        bad_since.push_back(e.second);
    return common_cause(bad_since, min_count, window);
}

}  // namespace sensor_watchdog
