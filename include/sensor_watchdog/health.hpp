#pragma once
// ROS에 의존하지 않는 순수 판정 로직.
// 1) Health        : 관측(그래프에 노드·발행자가 있는가, 드라이버 /diagnostics 가 정상인가) → 고장 코드
// 2) Ladder        : 고장 코드 + 시간 → 지금 뭘 할 차례인지(유예/대기/행동/포기)
// 3) common_cause  : 여러 센서가 동시에 죽었는지
//
// 센서 토픽은 구독하지 않는다. 실차에서 구독자가 하나 더 붙으면 라이다 발행 주기가 떨어지는
// 현상이 있어, 감시 근거는 DDS 그래프(디스커버리 정보)와 드라이버가 내는 /diagnostics 로 제한한다.
// 그래서 "발행자는 살아 있는데 데이터만 멈춤"은 드라이버가 diag 를 내지 않으면 잡을 수 없다.
//
// 이 파일은 test/test_health.cpp 로 ROS 없이 단독 테스트된다.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace sensor_watchdog
{

// ---------------------------------------------------------------- 설정

struct SensorConfig
{
    std::string id;

    std::size_t max_publishers = 1;   // 이보다 많으면 DUPLICATE (드라이버가 두 번 떠 있음)

    bool diag_checked = false;        // diag_names 가 설정돼 있는가 (없으면 그래프만 본다)
    double diag_timeout_sec = 3.0;    // 마지막 diag 보고 후 허용 시간. 넘으면 DIAG_STALE
    int diag_error_level = 2;         // 이 level 이상이면 DIAG_ERROR (0 OK, 1 WARN, 2 ERROR, 3 STALE)
};

// ---------------------------------------------------------------- 고장 코드

enum class Fault
{
    NONE,
    NODE_NOT_VISIBLE,   // node_name 이 그래프에 없다 (드라이버 프로세스가 죽음)
    NO_PUBLISHER,       // 토픽에 발행자가 없다 (드라이버가 죽었거나 아직 안 떴음)
    DIAG_STALE,         // 드라이버 diag 보고가 끊겼다 (프로세스는 있는데 멈춤)
    DIAG_ERROR,         // 드라이버 diag 가 ERROR 다 (드라이버 스스로 고장을 보고)
    DUPLICATE           // 같은 토픽 발행자 또는 같은 이름 노드가 둘 이상 (드라이버가 중복으로 떠 있음)
};

inline const char *to_string(Fault f)
{
    switch (f) {
    case Fault::NONE:             return "NONE";
    case Fault::NODE_NOT_VISIBLE: return "NODE_NOT_VISIBLE";
    case Fault::NO_PUBLISHER:     return "NO_PUBLISHER";
    case Fault::DIAG_STALE:       return "DIAG_STALE";
    case Fault::DIAG_ERROR:       return "DIAG_ERROR";
    case Fault::DUPLICATE:        return "DUPLICATE";
    }
    return "?";
}

constexpr Fault LAST_FAULT = Fault::DUPLICATE;

// "A+B+C" 형태로 합친다. 비어 있으면 "NONE".
inline std::string join_faults(const std::vector<Fault> &faults)
{
    if (faults.empty())
        return "NONE";
    std::string out;
    for (Fault f : faults)
        out += std::string(out.empty() ? "" : "+") + to_string(f);
    return out;
}

inline bool has_fault(const std::vector<Fault> &faults, Fault f)
{
    return std::find(faults.begin(), faults.end(), f) != faults.end();
}

// ROS 그래프에서 본 정보. 노드 쪽에서 채워서 넘긴다. 구독 없이 디스커버리 정보만으로 얻는다.
struct GraphView
{
    bool node_checked = false;   // node_name이 설정돼 있는가
    std::size_t node_count = 1;  // get_node_names() 에 node_name 이 몇 번 나오는가 (0 = 없음, 2+ = 중복)
    std::size_t publishers = 1;  // count_publishers(topic)

    bool node_visible() const { return !node_checked || node_count > 0; }
};

// ---------------------------------------------------------------- Health

class Health
{
public:
    explicit Health(SensorConfig cfg) : cfg_(std::move(cfg)) {}

    const SensorConfig &config() const { return cfg_; }

    // 재시작 직후 등, 관측을 처음부터 다시 쌓을 때.
    // diag 는 "방금 본 것"으로 취급해 diag_timeout_sec 동안 보고가 없어야 STALE 이 된다.
    void reset(double now)
    {
        last_diag_ = now;
        level_ = 0;
        message_.clear();
        received_ = false;
    }

    // 이 센서와 관련된 /diagnostics 상태를 받았을 때. level 은 그 묶음 중 가장 나쁜 값.
    void on_diag(double now, int level, std::string message)
    {
        last_diag_ = now;
        level_ = level;
        message_ = std::move(message);
        received_ = true;
        ++count_;
    }

    // 지금 해당되는 고장을 심한 것부터.
    // - 노드 없음 / 발행자 없음은 데이터 출처가 없는 상태라 그것 하나만 돌려준다.
    // - 발행자가 있으면 중복(DUPLICATE)과 diag 상태(STALE 또는 ERROR)를 모은다.
    std::vector<Fault> diagnose_all(double now, const GraphView &g) const
    {
        if (!g.node_visible())
            return {Fault::NODE_NOT_VISIBLE};

        if (g.publishers == 0)
            return {Fault::NO_PUBLISHER};

        std::vector<Fault> out;

        if (g.publishers > cfg_.max_publishers || (g.node_checked && g.node_count > 1))
            out.push_back(Fault::DUPLICATE);

        if (cfg_.diag_checked) {
            if (now - last_diag_ > cfg_.diag_timeout_sec)
                out.push_back(Fault::DIAG_STALE);
            else if (level_ >= cfg_.diag_error_level)
                out.push_back(Fault::DIAG_ERROR);
        }

        return out;
    }

    // 가장 심한 고장 하나. 없으면 NONE.
    Fault diagnose(double now, const GraphView &g) const
    {
        const auto all = diagnose_all(now, g);
        return all.empty() ? Fault::NONE : all.front();
    }

    double diag_age(double now) const { return now - last_diag_; }
    int diag_level() const { return level_; }
    const std::string &diag_message() const { return message_; }
    bool received() const { return received_; }
    uint64_t count() const { return count_; }

private:
    SensorConfig cfg_;

    double last_diag_ = 0.0;
    int level_ = 0;
    std::string message_;
    bool received_ = false;
    uint64_t count_ = 0;
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
