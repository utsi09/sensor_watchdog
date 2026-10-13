// sensor_watchdog: ROS 2 Humble 센서 감시 노드 (ROS 연동 부분)
//
// 하는 일
//   1. 설정된 토픽을 GenericSubscription으로 받아 header.stamp와 크기만 읽는다 (역직렬화 안 함)
//   2. 매 메시지를 원시 CSV로 남긴다 (raw_log_dir)
//   3. 200ms마다 판정하고 /diagnostics 로 상태를 발행한다
//   4. 고장이 지속되면 사다리(lifecycle → service → power)를 한 칸씩 올라간다
//   5. 단, 안전 게이트(gate_topic)가 허용하고 공통 원인이 아닐 때만 행동한다
//   6. systemd에 WATCHDOG=1 핑, MCU에 UDP 하트비트
//   7. 모든 행동과 복구 시간을 events.csv 에 남긴다
//
// 와치독은 IPC 에 하나만 둔다. 원격 장비의 센서는 systemctl_cmd 를 SSH 로 지정해 그 장비의 systemd 를 호출한다.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

#include "sensor_watchdog/cdr_header.hpp"
#include "sensor_watchdog/health.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

using namespace std::chrono_literals;
using namespace sensor_watchdog;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using Transition = lifecycle_msgs::msg::Transition;

namespace
{

double steady_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string wall_iso()
{
    char buf[32];
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
    return buf;
}

std::string expand_home(std::string path)
{
    if (!path.empty() && path.front() == '~') {
        const char *home = std::getenv("HOME");
        if (home)
            path = std::string(home) + path.substr(1);
    }
    return path;
}

// 셸을 거치지 않고 인자를 그대로 실행. 종료 코드 반환(항상 0 이상).
int run_command(std::vector<std::string> args)
{
    std::vector<char *> argv;
    for (auto &a : args)
        argv.push_back(a.data());
    argv.push_back(nullptr);

    pid_t pid = 0;
    const int err = posix_spawnp(&pid, argv[0], nullptr, nullptr, argv.data(), environ);
    if (err != 0)
        return 200 + (err % 50);   // 실행 자체 실패

    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR)
            return 255;
    }
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    if (WIFSIGNALED(status))
        return 128 + WTERMSIG(status);
    return 255;
}

// systemd Type=notify 용. NOTIFY_SOCKET 없으면 아무것도 안 함.
class SdNotify
{
public:
    SdNotify()
    {
        const char *path = std::getenv("NOTIFY_SOCKET");
        if (!path || (path[0] != '/' && path[0] != '@'))
            return;

        fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd_ < 0)
            return;

        addr_.sun_family = AF_UNIX;
        std::strncpy(addr_.sun_path, path, sizeof(addr_.sun_path) - 1);
        len_ = offsetof(sockaddr_un, sun_path) + std::strlen(path);
        if (path[0] == '@')
            addr_.sun_path[0] = '\0';
    }

    ~SdNotify()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    void send(const char *msg)
    {
        if (fd_ < 0)
            return;
        ::sendto(fd_, msg, std::strlen(msg), MSG_NOSIGNAL,
                 reinterpret_cast<sockaddr *>(&addr_), static_cast<socklen_t>(len_));
    }

private:
    int fd_ = -1;
    sockaddr_un addr_{};
    std::size_t len_ = 0;
};

// MCU 하트비트. DDS와 무관한 UDP 한 줄.
class UdpHeartbeat
{
public:
    UdpHeartbeat(const std::string &host, int port)
    {
        if (host.empty())
            return;

        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1)
            throw std::runtime_error("heartbeat_host must be an IPv4 address: " + host);

        fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (fd_ < 0)
            throw std::runtime_error("heartbeat socket failed");
    }

    ~UdpHeartbeat()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool enabled() const { return fd_ >= 0; }

    void send(const std::string &line)
    {
        if (fd_ < 0)
            return;
        ::sendto(fd_, line.data(), line.size(), MSG_NOSIGNAL,
                 reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
    }

private:
    int fd_ = -1;
    sockaddr_in addr_{};
};

// 진행 중인 복구 행동. result < 0 이면 아직 진행 중.
struct Action
{
    Step step;
    double started = 0.0;
    std::string describe;
    std::shared_ptr<std::atomic<int>> result;
};

struct Sensor
{
    std::string id;
    std::string topic;
    std::string type;
    std::string node_name;
    std::string lifecycle_node;
    std::string unit;
    std::vector<std::string> power_cmd;
    std::vector<std::string> systemctl_cmd;   // 비어 있으면 전역 systemctl_cmd 사용

    Health health;
    Ladder ladder;

    rclcpp::GenericSubscription::SharedPtr sub;
    rclcpp::Client<ChangeState>::SharedPtr lc_client;

    std::optional<Action> action;
    std::ofstream raw;

    std::string state;             // 마지막으로 보고한 상태 문자열
    Fault fault = Fault::NONE;
    double fault_started = -1.0;   // 복구 시간 측정용

    Sensor(SensorConfig c, Policy p) : health(std::move(c)), ladder(std::move(p)) {}
};

}  // namespace

class Watchdog : public rclcpp::Node
{
public:
    Watchdog() : Node("sensor_watchdog")
    {
        // ---------------- 전역 설정
        dry_run_ = declare_parameter<bool>("dry_run", true);

        raw_log_dir_ = expand_home(
            declare_parameter<std::string>("raw_log_dir", "~/.local/state/sensor_watchdog"));

        // 안전 게이트: 이 토픽의 첫 정수 필드가 gate_ok_value 이면 복구 행동을 허용한다.
        // 예) /ad_can/gear 의 기어 값이 P(파킹)일 때만.
        gate_topic_ = declare_parameter<std::string>("gate_topic", "");
        gate_type_ = declare_parameter<std::string>("gate_type", "std_msgs/msg/Bool");
        gate_has_header_ = declare_parameter<bool>("gate_has_header", false);
        gate_field_bytes_ = declare_parameter<int>("gate_field_bytes", 1);
        gate_ok_value_ = declare_parameter<int>("gate_ok_value", 1);
        gate_timeout_ = declare_parameter<double>("gate_timeout_sec", 1.0);
        // 고장이 이 시간 이상 지속되면 게이트를 무시하고 복구한다. 음수면 항상 게이트를 따른다.
        gate_bypass_after_ = declare_parameter<double>("gate_bypass_after_sec", 2.0);
        gate_reliable_ = declare_parameter<bool>("gate_reliable", true);

        if (gate_field_bytes_ != 1 && gate_field_bytes_ != 2 &&
            gate_field_bytes_ != 4 && gate_field_bytes_ != 8)
        {
            throw std::runtime_error("gate_field_bytes must be 1, 2, 4 or 8");
        }

        common_cause_min_ = declare_parameter<int>("common_cause_min", 3);
        common_cause_window_ = declare_parameter<double>("common_cause_window_sec", 3.0);

        action_timeout_ = declare_parameter<double>("action_timeout_sec", 30.0);

        systemctl_cmd_ = declare_parameter<std::vector<std::string>>(
            "systemctl_cmd", std::vector<std::string>{"systemctl", "--user"});

        const auto hb_host = declare_parameter<std::string>("heartbeat_host", "");
        const int hb_port = declare_parameter<int>("heartbeat_port", 5005);

        Policy base;
        base.grace_sec = declare_parameter<double>("startup_grace_sec", 10.0);
        base.hold_sec = declare_parameter<double>("fault_hold_sec", 2.0);

        // 복구 행동을 하는 고장 종류. 여기 없는 고장은 기록과 /diagnostics 보고만 한다.
        act_on_faults_ = declare_parameter<std::vector<std::string>>(
            "act_on_faults",
            std::vector<std::string>{"RX_TIMEOUT", "NO_PUBLISHER", "NODE_NOT_VISIBLE"});

        // 애매한 고장(주기 저하, 타임스탬프 이상). 더 오래 지속될 때만 행동한다.
        act_on_faults_slow_ = declare_parameter<std::vector<std::string>>(
            "act_on_faults_slow",
            std::vector<std::string>{"LOW_RX_HZ", "STAMP_NOT_ADVANCING", "STAMP_INVALID_OR_BACKWARD"});
        slow_fault_hold_ = declare_parameter<double>("slow_fault_hold_sec", 10.0);

        for (const auto *list : {&act_on_faults_, &act_on_faults_slow_}) {
            for (const auto &name : *list) {
                bool known = false;
                for (int i = 0; i <= static_cast<int>(Fault::LOW_HZ); ++i)
                    known = known || (name == to_string(static_cast<Fault>(i)));
                if (!known)
                    throw std::runtime_error("act_on_faults: unknown fault '" + name + "'");
            }
        }
        if (!std::isfinite(slow_fault_hold_) || slow_fault_hold_ < 0)
            throw std::runtime_error("slow_fault_hold_sec must be >= 0");
        base.cooldown_sec = declare_parameter<double>("cooldown_sec", 30.0);
        base.attempts_per_step = declare_parameter<int>("attempts_per_step", 2);
        base.healthy_reset_sec = declare_parameter<double>("healthy_reset_sec", 600.0);

        const auto ladder_names = declare_parameter<std::vector<std::string>>(
            "ladder", std::vector<std::string>{"service"});

        for (const auto &name : ladder_names) {
            Step s;
            if (!parse_step(name, s))
                throw std::runtime_error("ladder: unknown step '" + name + "'");
            base.ladder.push_back(s);
        }

        if (!std::isfinite(base.grace_sec) || base.grace_sec < 0 ||
            !std::isfinite(base.hold_sec) || base.hold_sec < 0 ||
            !std::isfinite(base.cooldown_sec) || base.cooldown_sec < 0 ||
            base.attempts_per_step < 1 ||
            !std::isfinite(base.healthy_reset_sec) || base.healthy_reset_sec < 0 ||
            action_timeout_ <= 0 || gate_timeout_ <= 0 ||
            systemctl_cmd_.empty())
        {
            throw std::runtime_error("invalid global parameters");
        }

        if (!raw_log_dir_.empty()) {
            ::mkdir(raw_log_dir_.c_str(), 0700);
            events_.open(raw_log_dir_ + "/events.csv", std::ios::app);
            if (!events_)
                throw std::runtime_error("cannot open " + raw_log_dir_ + "/events.csv");
            if (events_.tellp() == 0)
                events_ << "wall,steady,sensor,event,fault,step,code,detail\n";
        }

        heartbeat_ = std::make_unique<UdpHeartbeat>(hb_host, hb_port);

        // ---------------- 센서
        const auto ids = declare_parameter<std::vector<std::string>>(
            "sensors", std::vector<std::string>{});
        if (ids.empty())
            throw std::runtime_error("'sensors' is empty");

        for (const auto &id : ids)
            add_sensor(id, base);

        // ---------------- 안전 게이트
        if (!gate_topic_.empty()) {
            rclcpp::QoS gate_qos{rclcpp::KeepLast(10)};
            if (gate_reliable_) gate_qos.reliable(); else gate_qos.best_effort();

            gate_sub_ = create_generic_subscription(
                gate_topic_, gate_type_, gate_qos,
                [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    const auto &rcl = msg->get_rcl_serialized_message();
                    const auto v = read_first_int(
                        rcl.buffer, rcl.buffer_length,
                        gate_has_header_, static_cast<std::size_t>(gate_field_bytes_));
                    if (!v)
                        return;   // 파싱 실패는 "허용 아님"으로 남는다

                    const uint64_t mask = (gate_field_bytes_ == 8)
                        ? ~uint64_t(0)
                        : ((uint64_t(1) << (8 * gate_field_bytes_)) - 1);
                    gate_last_value_ = *v;
                    gate_value_ = (*v == (static_cast<uint64_t>(gate_ok_value_) & mask));
                    gate_rx_ = steady_seconds();
                });
        }

        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(10));

        timer_ = create_wall_timer(200ms, [this] { tick(); });

        sd_.send("READY=1");

        RCLCPP_INFO(get_logger(),
            "started: dry_run=%s gate=%s ladder=%zu steps heartbeat=%s raw_log=%s",
            dry_run_ ? "true" : "false",
            gate_topic_.empty() ? "(none: always allowed)" : gate_topic_.c_str(),
            base.ladder.size(),
            heartbeat_->enabled() ? "on" : "off",
            raw_log_dir_.empty() ? "off" : raw_log_dir_.c_str());
    }

private:
    template<typename T>
    T param(const std::string &id, const std::string &key, T fallback)
    {
        return declare_parameter<T>(id + "." + key, fallback);
    }

    void add_sensor(const std::string &id, const Policy &base)
    {
        SensorConfig c;
        c.id = id;
        c.timeout_sec = param<double>(id, "timeout_sec", 1.0);
        c.stamp_timeout_sec = param<double>(id, "stamp_timeout_sec", 1.0);
        c.min_hz = param<double>(id, "min_hz", 0.0);
        c.hz_window_sec = param<double>(id, "hz_window_sec", 5.0);
        c.min_bytes = param<int>(id, "min_bytes", 1);
        c.check_stamp_progress = param<bool>(id, "check_stamp_progress", true);
        c.check_stamp_age = param<bool>(id, "check_stamp_age", false);
        c.max_age_sec = param<double>(id, "max_age_sec", 0.5);

        for (double v : {c.timeout_sec, c.stamp_timeout_sec, c.hz_window_sec, c.max_age_sec}) {
            if (!std::isfinite(v) || v <= 0)
                throw std::runtime_error(id + ": durations must be positive");
        }
        if (!std::isfinite(c.min_hz) || c.min_hz < 0 || c.min_bytes < 0)
            throw std::runtime_error(id + ": invalid min_hz/min_bytes");

        // 이 센서에서 실제로 가능한 칸만 남긴다
        Policy p = base;
        const auto lifecycle_node = param<std::string>(id, "lifecycle_node", "");
        const auto unit = param<std::string>(id, "unit", "");
        const auto power_cmd = param<std::vector<std::string>>(
            id, "power_cmd", std::vector<std::string>{});

        p.ladder.clear();
        for (Step s : base.ladder) {
            if (s == Step::LIFECYCLE && lifecycle_node.empty()) continue;
            if (s == Step::SERVICE && unit.empty()) continue;
            if (s == Step::POWER && power_cmd.empty()) continue;
            p.ladder.push_back(s);
        }

        auto s = std::make_unique<Sensor>(c, p);
        s->id = id;
        s->topic = param<std::string>(id, "topic", "");
        s->type = param<std::string>(id, "type", "");
        s->node_name = param<std::string>(id, "node_name", "");
        s->lifecycle_node = lifecycle_node;
        s->unit = unit;
        s->power_cmd = power_cmd;

        // 센서별 재시작 명령 앞부분. 원격 장비의 유닛이면
        // ["ssh", "-o", "BatchMode=yes", "orin", "systemctl", "--user"] 처럼 준다.
        s->systemctl_cmd = param<std::vector<std::string>>(
            id, "systemctl_cmd", std::vector<std::string>{});

        if (s->topic.empty() || s->topic.front() != '/')
            throw std::runtime_error(id + ": topic must start with '/'");
        if (s->type.empty())
            throw std::runtime_error(id + ": type is required (e.g. sensor_msgs/msg/PointCloud2)");
        if (!s->node_name.empty() && s->node_name.front() != '/')
            throw std::runtime_error(id + ": node_name must start with '/'");
        if (!s->lifecycle_node.empty() && s->lifecycle_node.front() != '/')
            throw std::runtime_error(id + ": lifecycle_node must start with '/'");

        // QoS
        const int depth = param<int>(id, "qos.depth", 5);
        const auto reliability = param<std::string>(id, "qos.reliability", "best_effort");
        const auto durability = param<std::string>(id, "qos.durability", "volatile");
        if (depth < 1)
            throw std::runtime_error(id + ": qos.depth must be >= 1");

        rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(depth))};
        if (reliability == "best_effort")  qos.best_effort();
        else if (reliability == "reliable") qos.reliable();
        else throw std::runtime_error(id + ": invalid qos.reliability");
        if (durability == "volatile")             qos.durability_volatile();
        else if (durability == "transient_local") qos.transient_local();
        else throw std::runtime_error(id + ": invalid qos.durability");

        // 원시 로그
        if (!raw_log_dir_.empty()) {
            s->raw.open(raw_log_dir_ + "/" + id + ".csv", std::ios::app);
            if (!s->raw)
                throw std::runtime_error(id + ": cannot open raw log");
            if (s->raw.tellp() == 0)
                s->raw << "steady,ros_now,stamp_ns,bytes\n";
        }

        if (!s->lifecycle_node.empty())
            s->lc_client = create_client<ChangeState>(s->lifecycle_node + "/change_state");

        const double t = steady_seconds();
        s->health.reset(t);
        s->ladder.reset(t);

        Sensor *ptr = s.get();
        s->sub = create_generic_subscription(
            s->topic, s->type, qos,
            [this, ptr](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                on_serialized(*ptr, *msg);
            });

        std::string ladder_str;
        for (Step st : p.ladder)
            ladder_str += std::string(ladder_str.empty() ? "" : ">") + to_string(st);

        RCLCPP_INFO(get_logger(), "[%s] topic=%s type=%s ladder=%s",
            id.c_str(), s->topic.c_str(), s->type.c_str(),
            ladder_str.empty() ? "(monitor only)" : ladder_str.c_str());

        sensors_.push_back(std::move(s));
    }

    void on_serialized(Sensor &s, const rclcpp::SerializedMessage &msg)
    {
        const auto &rcl = msg.get_rcl_serialized_message();
        const double t = steady_seconds();
        const double ros_now = now().seconds();

        int32_t sec = -1;
        uint32_t nsec = 0;
        if (const auto st = read_header_stamp(rcl.buffer, rcl.buffer_length)) {
            sec = st->sec;
            nsec = st->nanosec;
        }

        s.health.on_message(t, ros_now, sec, nsec, rcl.buffer_length);

        if (s.raw.is_open()) {
            s.raw << t << ',' << ros_now << ','
                  << (int64_t(sec) * 1000000000LL + int64_t(nsec)) << ','
                  << rcl.buffer_length << '\n';
        }
    }

    // ---------------- 복구 행동

    void spawn_async(std::vector<std::string> args, std::shared_ptr<std::atomic<int>> result)
    {
        std::thread([args = std::move(args), result] {
            result->store(run_command(args));
        }).detach();
    }

    void lifecycle_cycle(Sensor &s, std::shared_ptr<std::atomic<int>> result)
    {
        auto client = s.lc_client;
        if (!client || !client->service_is_ready()) {
            result->store(2);   // lifecycle 서비스 없음
            return;
        }

        auto deact = std::make_shared<ChangeState::Request>();
        deact->transition.id = Transition::TRANSITION_DEACTIVATE;

        // deactivate 결과는 무시한다(이미 inactive일 수 있음). activate 결과만 본다.
        client->async_send_request(deact,
            [client, result](rclcpp::Client<ChangeState>::SharedFuture) {
                auto act = std::make_shared<ChangeState::Request>();
                act->transition.id = Transition::TRANSITION_ACTIVATE;
                client->async_send_request(act,
                    [result](rclcpp::Client<ChangeState>::SharedFuture f) {
                        result->store(f.get()->success ? 0 : 3);
                    });
            });
    }

    void start_action(Sensor &s, double t)
    {
        Action a;
        a.step = s.ladder.current();
        a.started = t;
        a.result = std::make_shared<std::atomic<int>>(-1);

        const std::string timeout_s = std::to_string(static_cast<int>(action_timeout_)) + "s";

        switch (a.step) {
        case Step::LIFECYCLE:
            a.describe = "lifecycle " + s.lifecycle_node;
            lifecycle_cycle(s, a.result);
            break;

        case Step::SERVICE: {
            std::vector<std::string> cmd = {"timeout", "--signal=TERM", "--kill-after=5s", timeout_s};
            const auto &prefix = s.systemctl_cmd.empty() ? systemctl_cmd_ : s.systemctl_cmd;
            cmd.insert(cmd.end(), prefix.begin(), prefix.end());
            cmd.push_back("restart");
            cmd.push_back(s.unit);
            a.describe = "restart " + s.unit;
            spawn_async(cmd, a.result);
            break;
        }

        case Step::POWER: {
            std::vector<std::string> cmd = {"timeout", "--signal=TERM", "--kill-after=5s", timeout_s};
            cmd.insert(cmd.end(), s.power_cmd.begin(), s.power_cmd.end());
            cmd.push_back(s.id);
            a.describe = "power " + s.power_cmd.front();
            spawn_async(cmd, a.result);
            break;
        }
        }

        s.ladder.on_action_started(t);
        s.action = a;

        RCLCPP_ERROR(get_logger(), "[%s] ACTION %s (step %zu, attempt %d, total %d) because %s",
            s.id.c_str(), a.describe.c_str(), s.ladder.step_index() + 1,
            s.ladder.attempts(), s.ladder.total_actions(), to_string(s.fault));

        event(s, "action_start", to_string(a.step), -1, a.describe);
    }

    // ---------------- 주기 검사

    void tick()
    {
        const double t = steady_seconds();
        sd_.send("WATCHDOG=1");

        bool any_node_check = false;
        for (const auto &s : sensors_)
            any_node_check = any_node_check || !s->node_name.empty();
        const auto nodes = any_node_check ? get_node_names() : std::vector<std::string>{};

        const bool gate_ok =
            gate_topic_.empty() || (gate_value_ && t - gate_rx_ <= gate_timeout_);

        std::vector<double> bad_since;
        for (const auto &s : sensors_)
            bad_since.push_back(s->ladder.bad_since());
        const bool cc = common_cause(bad_since, common_cause_min_, common_cause_window_);

        int ok_count = 0;
        int worst = 0;

        for (auto &entry : sensors_) {
            Sensor &s = *entry;

            // 진행 중인 행동 마무리
            if (s.action) {
                int r = s.action->result->load();
                if (r < 0 && t - s.action->started > action_timeout_ + 5.0)
                    r = 124;   // 응답 없음

                if (r < 0) {
                    report(s, "ACTING", s.fault);
                    worst = 2;
                    continue;
                }

                RCLCPP_WARN(get_logger(), "[%s] action done exit=%d; re-checking after grace",
                    s.id.c_str(), r);
                event(s, "action_done", to_string(s.action->step), r, s.action->describe);

                s.action.reset();
                s.health.reset(t);
                s.ladder.on_action_finished(t);
            }

            GraphView g;
            g.node_checked = !s.node_name.empty();
            g.node_visible = !g.node_checked ||
                std::find(nodes.begin(), nodes.end(), s.node_name) != nodes.end();
            g.publishers = count_publishers(s.topic);
            g.matched = s.sub->get_publisher_count();

            const Fault fault = s.health.diagnose(t, g);
            const std::string fault_name = to_string(fault);
            const bool hard = std::find(act_on_faults_.begin(), act_on_faults_.end(),
                                        fault_name) != act_on_faults_.end();
            const bool slow = std::find(act_on_faults_slow_.begin(), act_on_faults_slow_.end(),
                                        fault_name) != act_on_faults_slow_.end();

            // 확실한 끊김은 fault_hold_sec, 애매한 고장은 slow_fault_hold_sec 만큼 참는다
            const double hold = hard ? s.ladder.policy().hold_sec : slow_fault_hold_;
            const Verdict v = s.ladder.evaluate(t, fault, hold);

            std::string state = to_string(v);

            if (v == Verdict::HEALTHY) {
                ++ok_count;
                if (s.fault_started >= 0.0) {
                    event(s, "recovered", "", 0,
                        "after " + std::to_string(t - s.fault_started) + "s");
                    s.fault_started = -1.0;
                }
            }
            else if (v != Verdict::GRACE) {
                if (s.fault_started < 0.0)
                    s.fault_started = t;
                worst = 2;
            }
            else {
                worst = std::max(worst, 1);
            }

            const bool actionable = hard || slow;

            if (v == Verdict::ACT && !actionable) {
                state = "MONITOR_ONLY";
            }
            else if (v == Verdict::ACT) {
                // 고장이 충분히 오래 지속됐으면 센서는 이미 쓸모없는 상태다.
                // 이때는 게이트(기어 등)를 기다리지 않고 바로 복구한다.
                const bool gate_bypass =
                    gate_bypass_after_ >= 0.0 &&
                    s.ladder.bad_since() >= 0.0 &&
                    t - s.ladder.bad_since() >= gate_bypass_after_;

                if (cc)                               state = "BLOCKED_COMMON_CAUSE";
                else if (!gate_ok && !gate_bypass)    state = "BLOCKED_GATE";
                else if (dry_run_)                    state = std::string("DRY_RUN_WOULD_") + to_string(s.ladder.current());
                else {
                    s.fault = fault;
                    if (!gate_ok && gate_bypass)
                        event(s, "gate_bypass", "", -1,
                            "fault persisted " + std::to_string(t - s.ladder.bad_since()) + "s");
                    start_action(s, t);
                    state = "ACTING";
                }
            }

            report(s, state, fault);
        }

        if (++tick_count_ % 5 == 0)
            publish_diagnostics(ok_count);

        if (heartbeat_->enabled()) {
            heartbeat_->send(
                "WD " + std::to_string(tick_count_) + " " +
                std::to_string(ok_count) + "/" + std::to_string(sensors_.size()) + " " +
                std::to_string(worst) + "\n");
        }

        for (auto &s : sensors_)
            if (s->raw.is_open()) s->raw.flush();
        if (events_.is_open())
            events_.flush();
    }

    // 상태가 바뀔 때만 로그와 이벤트를 남긴다
    void report(Sensor &s, const std::string &state, Fault fault)
    {
        if (state == s.state && fault == s.fault)
            return;

        if (state == "HEALTHY")
            RCLCPP_INFO(get_logger(), "[%s] HEALTHY", s.id.c_str());
        else if (state == "GRACE")
            RCLCPP_INFO(get_logger(), "[%s] GRACE", s.id.c_str());
        else if (state == "EXHAUSTED")
            RCLCPP_ERROR(get_logger(), "[%s] EXHAUSTED: %s — human needed",
                s.id.c_str(), to_string(fault));
        else
            RCLCPP_WARN(get_logger(), "[%s] %s: %s", s.id.c_str(), state.c_str(), to_string(fault));

        event(s, "state", state, -1, to_string(fault));
        s.state = state;
        s.fault = fault;
    }

    void event(Sensor &s, const std::string &kind, const std::string &step,
               int code, const std::string &detail)
    {
        if (!events_.is_open())
            return;
        events_ << wall_iso() << ',' << steady_seconds() << ',' << s.id << ','
                << kind << ',' << to_string(s.fault) << ',' << step << ','
                << code << ',' << detail << '\n';
    }

    void publish_diagnostics(int ok_count)
    {
        using diagnostic_msgs::msg::DiagnosticStatus;
        using diagnostic_msgs::msg::KeyValue;

        diagnostic_msgs::msg::DiagnosticArray arr;
        arr.header.stamp = now();

        const double t = steady_seconds();

        for (auto &entry : sensors_) {
            Sensor &s = *entry;
            DiagnosticStatus st;
            st.name = std::string(get_name()) + ": " + s.id;
            st.hardware_id = s.topic;

            // 플래너 기준: 지금 이 센서를 써도 되나
            if (s.state == "HEALTHY")      st.level = DiagnosticStatus::OK;
            else if (s.state == "GRACE")   st.level = DiagnosticStatus::WARN;
            else                           st.level = DiagnosticStatus::ERROR;

            st.message = s.state + (s.fault == Fault::NONE ? "" : std::string(": ") + to_string(s.fault));

            const auto kv = [&st](const std::string &k, const std::string &v) {
                KeyValue x; x.key = k; x.value = v; st.values.push_back(x);
            };
            kv("fault", to_string(s.fault));
            kv("state", s.state);
            kv("rx_hz", std::to_string(s.health.hz(t)));
            kv("age_sec", std::to_string(s.health.age()));
            kv("rx_count", std::to_string(s.health.count()));
            kv("step", s.ladder.policy().ladder.empty() ? "none" : to_string(s.ladder.current()));
            kv("attempts_at_step", std::to_string(s.ladder.attempts()));
            kv("total_actions", std::to_string(s.ladder.total_actions()));
            kv("exhausted", s.ladder.exhausted() ? "true" : "false");
            kv("dry_run", dry_run_ ? "true" : "false");

            arr.status.push_back(st);
        }

        DiagnosticStatus sum;
        sum.name = std::string(get_name()) + ": summary";
        sum.level = (ok_count == static_cast<int>(sensors_.size()))
            ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
        sum.message = std::to_string(ok_count) + "/" + std::to_string(sensors_.size()) + " healthy";
        {
            KeyValue x;
            x.key = "gate";
            if (gate_topic_.empty())
                x.value = "not configured (always allowed)";
            else if (t - gate_rx_ > gate_timeout_)
                x.value = "no message (blocked)";
            else
                x.value = std::string(gate_value_ ? "allowed" : "blocked") +
                          " value=" + std::to_string(gate_last_value_);
            sum.values.push_back(x);
        }
        arr.status.push_back(sum);

        diag_pub_->publish(arr);
    }

    // ---------------- 멤버
    bool dry_run_ = true;
    std::string raw_log_dir_;
    std::string gate_topic_;
    std::string gate_type_;
    bool gate_has_header_ = false;
    int gate_field_bytes_ = 1;
    int gate_ok_value_ = 1;
    bool gate_reliable_ = true;
    double gate_timeout_ = 1.0;
    double gate_bypass_after_ = 5.0;
    int common_cause_min_ = 3;
    double common_cause_window_ = 3.0;
    double action_timeout_ = 30.0;
    std::vector<std::string> systemctl_cmd_;
    std::vector<std::string> act_on_faults_;
    std::vector<std::string> act_on_faults_slow_;
    double slow_fault_hold_ = 10.0;

    bool gate_value_ = false;
    uint64_t gate_last_value_ = 0;
    double gate_rx_ = -1e9;

    uint64_t tick_count_ = 0;

    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::ofstream events_;

    SdNotify sd_;
    std::unique_ptr<UdpHeartbeat> heartbeat_;

    rclcpp::GenericSubscription::SharedPtr gate_sub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        rclcpp::spin(std::make_shared<Watchdog>());
    }
    catch (const std::exception &e) {
        fprintf(stderr, "sensor_watchdog: %s\n", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
