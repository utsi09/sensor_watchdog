// sensor_watchdog: ROS 2 Humble 센서 감시 노드 (ROS 연동 부분)
//
// 하는 일
//   1. 센서 토픽은 구독하지 않는다. 0.2초마다 DDS 그래프에서 "노드가 보이는가", "토픽에 발행자가
//      몇 개인가"를 묻고, 드라이버가 /diagnostics 를 내면 그 보고(level, 끊김)를 본다.
//      → 실차에서 구독자가 하나 더 붙으면 라이다 발행 주기가 떨어지는 문제가 있어 데이터 경로에
//        아무것도 붙이지 않는다. 구독은 /diagnostics 하나(와 선택인 안전 게이트)뿐이다.
//   2. 판정 결과를 /diagnostics 로 발행한다 (1초마다)
//   3. 고장이 지속되면 사다리(lifecycle → service → power)를 한 칸씩 올라간다
//   4. 단, 안전 게이트(gate_topic)가 허용하고 공통 원인이 아닐 때만 행동한다
//      (공통 원인은 복구 단위=유닛 기준. 한 유닛을 공유하는 센서는 한 번만 재시작한다)
//   5. 중복 검사: 같은 토픽 발행자(또는 같은 이름 노드)가 둘 이상이면 DUPLICATE.
//      와치독이 직접 띄운 유닛이 중복의 한쪽이면 그 유닛만 끈다 (systemctl stop <unit>).
//      와치독이 띄운 게 아니면 보고만 한다. 남의 프로세스는 절대 죽이지 않는다.
//   6. systemd에 WATCHDOG=1 핑, MCU에 UDP 하트비트
//   7. 모든 상태 변화·행동·복구 시간을 events.csv 에 남긴다
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
#include <cstdio>
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
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

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

// CSV 로그를 append 로 연다. 비어 있으면 헤더를 쓴다.
bool open_csv(std::ofstream &f, const std::string &path, const char *header)
{
    f.open(path, std::ios::app);
    if (!f)
        return false;
    if (f.tellp() == 0)
        f << header << '\n';
    return true;
}

// 파일이 max_bytes 를 넘으면 <path>.1 로 밀어내고(이전 .1 은 덮어씀) 새로 시작한다.
void rotate_csv(std::ofstream &f, const std::string &path, const char *header, uint64_t max_bytes)
{
    if (!f.is_open() || max_bytes == 0)
        return;
    if (static_cast<uint64_t>(f.tellp()) < max_bytes)
        return;
    f.close();
    std::rename(path.c_str(), (path + ".1").c_str());
    open_csv(f, path, header);
}

constexpr const char *EVENTS_HEADER = "wall,steady,sensor,event,fault,step,code,detail";

// 진행 중인 복구 행동. result < 0 이면 아직 진행 중.
struct Action
{
    Step step;
    double started = 0.0;
    std::string describe;
    bool duplicate_stop = false;   // 사다리가 아니라 "와치독이 띄운 중복 유닛 끄기"
    std::shared_ptr<std::atomic<int>> result;
};

struct Sensor
{
    std::string id;
    std::string topic;
    std::string node_name;
    std::string lifecycle_node;
    std::string unit;
    std::vector<std::string> power_cmd;
    std::vector<std::string> systemctl_cmd;   // 비어 있으면 전역 systemctl_cmd 사용
    std::vector<std::string> diag_names;      // /diagnostics 에서 이 센서로 볼 status 이름 조각

    // 복구 단위 키. 같은 유닛(같은 systemctl_cmd + unit)을 쓰는 센서는 같은 키를 갖는다.
    // 라이다 4대가 한 런치에서 돌면 넷이 같은 키. 유닛이 없으면 센서 id.
    std::string domain;

    Health health;
    Ladder ladder;
    GraphView graph;               // 마지막 틱에서 본 그래프 (진단 출력용)

    rclcpp::Client<ChangeState>::SharedPtr lc_client;

    std::optional<Action> action;

    // 이 유닛의 현재 인스턴스를 와치독이 띄웠는가 (service/power 행동 완료 후 true).
    // 중복이 생겼을 때 "와치독이 띄운 것"만 끄기 위한 근거. 끄고 나면 false.
    bool started_by_us = false;

    std::string state;             // 마지막으로 보고한 상태 문자열
    std::string faults = "NONE";   // 지금 해당되는 고장 전부 ("A+B")
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

        // events.csv 저장 위치. "" 면 끔. (예전 이름 raw_log_dir 도 받는다)
        log_dir_ = expand_home(declare_parameter<std::string>("log_dir", ""));
        if (log_dir_.empty())
            log_dir_ = expand_home(
                declare_parameter<std::string>("raw_log_dir", "~/.local/state/sensor_watchdog"));
        else
            declare_parameter<std::string>("raw_log_dir", "");
        // events.csv 크기 상한(MB). 넘으면 .1 백업 하나만 남기고 새로 시작. 0 이면 상한 없음.
        {
            double mb = declare_parameter<double>("log_max_mb", -1.0);
            const double old = declare_parameter<double>("raw_log_max_mb", 20.0);
            if (mb < 0.0)
                mb = old;
            log_max_bytes_ = static_cast<uint64_t>(std::max(0.0, mb) * 1024.0 * 1024.0);
        }

        // 안전 게이트: 이 토픽의 첫 정수 필드가 gate_ok_value 이면 복구 행동을 허용한다.
        // 예) /vehicle/status/gear_status 의 기어 값이 P(파킹)일 때만.
        // 센서 토픽이 아닌 작은 상태 토픽 하나를 구독한다. "" 면 구독하지 않는다 (기본).
        gate_topic_ = declare_parameter<std::string>("gate_topic", "");
        gate_type_ = declare_parameter<std::string>("gate_type", "std_msgs/msg/Bool");
        {
            const auto prefix = declare_parameter<std::string>("gate_prefix", "");
            const bool has_header = declare_parameter<bool>("gate_has_header", false);
            if (prefix.empty())
                gate_prefix_ = has_header ? Prefix::HEADER : Prefix::NONE;
            else if (!parse_prefix(prefix, gate_prefix_))
                throw std::runtime_error("gate_prefix must be none, stamp or header");
        }
        gate_field_bytes_ = declare_parameter<int>("gate_field_bytes", 1);
        gate_ok_value_ = declare_parameter<int>("gate_ok_value", 1);
        gate_timeout_ = declare_parameter<double>("gate_timeout_sec", 1.0);
        gate_bypass_after_ = declare_parameter<double>("gate_bypass_after_sec", -1.0);
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
            std::vector<std::string>{"NODE_NOT_VISIBLE", "NO_PUBLISHER", "DIAG_STALE"});

        // 드라이버가 스스로 ERROR 라고 보고하는 경우. 주기 검사 같은 건 잠깐 튈 수 있어 더 오래 참는다.
        act_on_faults_slow_ = declare_parameter<std::vector<std::string>>(
            "act_on_faults_slow", std::vector<std::string>{"DIAG_ERROR"});
        slow_fault_hold_ = declare_parameter<double>("slow_fault_hold_sec", 5.0);

        // 중복(DUPLICATE)이 이만큼 지속되면, 와치독이 띄운 유닛이면 끈다. 재시작 직후엔
        // 옛 발행자가 디스커버리에서 빠지기까지 몇 초 동안 둘로 보이므로 짧게 잡지 않는다.
        duplicate_hold_ = declare_parameter<double>("duplicate_hold_sec", 5.0);

        for (const auto *list : {&act_on_faults_, &act_on_faults_slow_}) {
            for (const auto &name : *list) {
                bool known = false;
                for (int i = 0; i <= static_cast<int>(LAST_FAULT); ++i)
                    known = known || (name == to_string(static_cast<Fault>(i)));
                if (!known)
                    throw std::runtime_error("act_on_faults: unknown fault '" + name + "'");
            }
        }
        if (!std::isfinite(slow_fault_hold_) || slow_fault_hold_ < 0 ||
            !std::isfinite(duplicate_hold_) || duplicate_hold_ < 0)
            throw std::runtime_error("slow_fault_hold_sec / duplicate_hold_sec must be >= 0");
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

        if (!log_dir_.empty()) {
            ::mkdir(log_dir_.c_str(), 0700);
            events_path_ = log_dir_ + "/events.csv";
            if (!open_csv(events_, events_path_, EVENTS_HEADER))
                throw std::runtime_error("cannot open " + events_path_);
        }

        heartbeat_ = std::make_unique<UdpHeartbeat>(hb_host, hb_port);

        // ---------------- 센서
        const auto ids = declare_parameter<std::vector<std::string>>(
            "sensors", std::vector<std::string>{});
        if (ids.empty())
            throw std::runtime_error("'sensors' is empty");

        for (const auto &id : ids)
            add_sensor(id, base);

        // ---------------- 드라이버 진단 구독 (diag_names 를 쓰는 센서가 하나라도 있을 때만)
        bool any_diag = false;
        for (const auto &s : sensors_)
            any_diag = any_diag || !s->diag_names.empty();
        if (any_diag) {
            diag_sub_ = create_subscription<DiagnosticArray>(
                "/diagnostics", rclcpp::QoS(50),
                [this](DiagnosticArray::ConstSharedPtr arr) { on_diagnostics(*arr); });
        }

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
                        gate_prefix_, static_cast<std::size_t>(gate_field_bytes_));
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

        diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

        timer_ = create_wall_timer(200ms, [this] { tick(); });

        sd_.send("READY=1");

        RCLCPP_INFO(get_logger(),
            "started: dry_run=%s gate=%s ladder=%zu steps heartbeat=%s log=%s subscriptions=%s%s",
            dry_run_ ? "true" : "false",
            gate_topic_.empty() ? "(none: always allowed)" : gate_topic_.c_str(),
            base.ladder.size(),
            heartbeat_->enabled() ? "on" : "off",
            log_dir_.empty() ? "off" : log_dir_.c_str(),
            any_diag ? "/diagnostics" : "(none)",
            gate_topic_.empty() ? "" : " + gate");
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
        const auto diag_names = param<std::vector<std::string>>(
            id, "diag_names", std::vector<std::string>{});
        c.diag_checked = !diag_names.empty();
        c.diag_timeout_sec = param<double>(id, "diag_timeout_sec", 3.0);
        c.diag_error_level = param<int>(id, "diag_error_level", 2);
        const int max_pub = param<int>(id, "max_publishers", 1);

        if (!std::isfinite(c.diag_timeout_sec) || c.diag_timeout_sec <= 0)
            throw std::runtime_error(id + ": diag_timeout_sec must be positive");
        if (c.diag_error_level < 1 || c.diag_error_level > 3)
            throw std::runtime_error(id + ": diag_error_level must be 1 (WARN), 2 (ERROR) or 3 (STALE)");
        if (max_pub < 1)
            throw std::runtime_error(id + ": max_publishers must be >= 1");
        c.max_publishers = static_cast<std::size_t>(max_pub);

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
        s->node_name = param<std::string>(id, "node_name", "");
        s->lifecycle_node = lifecycle_node;
        s->unit = unit;
        s->power_cmd = power_cmd;
        s->diag_names = diag_names;

        // 센서별 재시작 명령 앞부분. 원격 장비의 유닛이면
        // ["ssh", "-o", "BatchMode=yes", "orin", "systemctl", "--user"] 처럼 준다.
        s->systemctl_cmd = param<std::vector<std::string>>(
            id, "systemctl_cmd", std::vector<std::string>{});

        if (s->unit.empty()) {
            s->domain = "sensor:" + id;
        } else {
            s->domain = "unit:";
            for (const auto &w : (s->systemctl_cmd.empty() ? systemctl_cmd_ : s->systemctl_cmd))
                s->domain += w + " ";
            s->domain += s->unit;
        }

        if (s->topic.empty() || s->topic.front() != '/')
            throw std::runtime_error(id + ": topic must start with '/'");
        if (!s->node_name.empty() && s->node_name.front() != '/')
            throw std::runtime_error(id + ": node_name must start with '/'");
        if (!s->lifecycle_node.empty() && s->lifecycle_node.front() != '/')
            throw std::runtime_error(id + ": lifecycle_node must start with '/'");

        if (!s->lifecycle_node.empty())
            s->lc_client = create_client<ChangeState>(s->lifecycle_node + "/change_state");

        const double t = steady_seconds();
        s->health.reset(t);
        s->ladder.reset(t);

        std::string ladder_str;
        for (Step st : p.ladder)
            ladder_str += std::string(ladder_str.empty() ? "" : ">") + to_string(st);

        RCLCPP_INFO(get_logger(), "[%s] topic=%s node=%s diag_names=%zu ladder=%s domain=%s",
            id.c_str(), s->topic.c_str(),
            s->node_name.empty() ? "(unchecked)" : s->node_name.c_str(),
            s->diag_names.size(),
            ladder_str.empty() ? "(monitor only)" : ladder_str.c_str(),
            s->domain.c_str());

        sensors_.push_back(std::move(s));
    }

    // 드라이버가 낸 /diagnostics. 센서마다 diag_names 조각이 이름에 들어 있는 status 를 모아
    // 가장 나쁜 level 을 그 센서의 보고로 삼는다. 와치독 자신의 출력은 건너뛴다.
    void on_diagnostics(const DiagnosticArray &arr)
    {
        const double t = steady_seconds();
        const std::string self_prefix = std::string(get_name()) + ": ";

        for (auto &entry : sensors_) {
            Sensor &s = *entry;
            if (s.diag_names.empty())
                continue;

            int worst = -1;
            std::string message;
            for (const auto &st : arr.status) {
                if (st.name.compare(0, self_prefix.size(), self_prefix) == 0)
                    continue;
                bool related = false;
                for (const auto &n : s.diag_names)
                    related = related || (st.name.find(n) != std::string::npos);
                if (!related)
                    continue;
                const int lv = static_cast<int>(st.level);
                if (lv > worst) {
                    worst = lv;
                    message = st.name + ": " + st.message;
                }
            }
            if (worst >= 0)
                s.health.on_diag(t, worst, message);
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

    std::vector<std::string> systemctl_prefix(const Sensor &s)
    {
        const std::string timeout_s = std::to_string(static_cast<int>(action_timeout_)) + "s";
        std::vector<std::string> cmd = {"timeout", "--signal=TERM", "--kill-after=5s", timeout_s};
        const auto &prefix = s.systemctl_cmd.empty() ? systemctl_cmd_ : s.systemctl_cmd;
        cmd.insert(cmd.end(), prefix.begin(), prefix.end());
        return cmd;
    }

    void start_action(Sensor &s, double t)
    {
        Action a;
        a.step = s.ladder.current();
        a.started = t;
        a.result = std::make_shared<std::atomic<int>>(-1);

        switch (a.step) {
        case Step::LIFECYCLE:
            a.describe = "lifecycle " + s.lifecycle_node;
            lifecycle_cycle(s, a.result);
            break;

        case Step::SERVICE: {
            auto cmd = systemctl_prefix(s);
            cmd.push_back("restart");
            cmd.push_back(s.unit);
            a.describe = "restart " + s.unit;
            spawn_async(cmd, a.result);
            break;
        }

        case Step::POWER: {
            const std::string timeout_s = std::to_string(static_cast<int>(action_timeout_)) + "s";
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

    // 와치독이 띄운 유닛이 중복의 한쪽일 때 그 유닛만 끈다. systemd 는 그 유닛이 띄운 프로세스만
    // 알고 있으므로(cgroup) 사람이 터미널에서 따로 띄운 쪽은 건드리지 않는다.
    void start_duplicate_stop(Sensor &s, double t)
    {
        Action a;
        a.step = Step::SERVICE;
        a.started = t;
        a.duplicate_stop = true;
        a.result = std::make_shared<std::atomic<int>>(-1);
        auto cmd = systemctl_prefix(s);
        cmd.push_back("stop");
        cmd.push_back(s.unit);
        a.describe = "stop " + s.unit + " (duplicate, started by watchdog)";
        spawn_async(cmd, a.result);
        s.action = a;

        // 이 유닛은 이제 와치독이 띄운 인스턴스가 아니다 (형제도 같은 유닛)
        s.started_by_us = false;
        for (Sensor *o : siblings(s))
            o->started_by_us = false;

        RCLCPP_ERROR(get_logger(), "[%s] DUPLICATE: %zu publishers on %s; stopping the unit we started (%s)",
            s.id.c_str(), s.graph.publishers, s.topic.c_str(), s.unit.c_str());
        event(s, "duplicate_stop", "service", -1, a.describe);
    }

    // 같은 복구 단위(유닛)를 공유하는 다른 센서들
    std::vector<Sensor *> siblings(const Sensor &s)
    {
        std::vector<Sensor *> out;
        for (auto &o : sensors_)
            if (o.get() != &s && o->domain == s.domain)
                out.push_back(o.get());
        return out;
    }

    // 같은 유닛에서 이미 행동이 진행 중인가
    bool domain_acting(const Sensor &s)
    {
        for (Sensor *o : siblings(s))
            if (o->action)
                return true;
        return false;
    }

    // 유닛이 재시작되면 그 유닛의 다른 센서도 함께 끊기고 다시 올라온다.
    // 그 센서들의 관측을 비우고 유예·쿨다운을 준다. 사다리 시도 횟수는 올리지 않는다.
    void grace_siblings(Sensor &s, double t, const char *why)
    {
        for (Sensor *o : siblings(s)) {
            o->health.reset(t);
            o->ladder.defer(t);
            event(*o, "sibling_grace", "", -1, std::string(why) + " by " + s.id);
        }
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

        // 공통 원인은 센서 수가 아니라 복구 단위(유닛) 수로 센다.
        // 한 런치의 라이다 4대가 같이 죽은 것은 그 유닛 하나의 고장이다.
        std::vector<std::pair<std::string, double>> domains;
        for (const auto &s : sensors_)
            domains.emplace_back(s->domain, s->ladder.bad_since());
        const bool cc = common_cause_by_domain(domains, common_cause_min_, common_cause_window_);

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
                    report(s, s.action->duplicate_stop ? "STOPPING_DUPLICATE" : "ACTING", s.fault);
                    worst = 2;
                    continue;
                }

                RCLCPP_WARN(get_logger(), "[%s] action done exit=%d; re-checking after grace",
                    s.id.c_str(), r);
                event(s, "action_done", to_string(s.action->step), r, s.action->describe);

                const Step done_step = s.action->step;
                const bool was_stop = s.action->duplicate_stop;
                s.action.reset();
                s.health.reset(t);
                if (was_stop) {
                    s.ladder.defer(t);              // 사다리 시도 횟수는 올리지 않는다
                } else {
                    s.ladder.on_action_finished(t);
                    if (done_step != Step::LIFECYCLE) {
                        // 이 유닛의 지금 인스턴스는 와치독이 띄운 것이다 (형제도 같은 유닛)
                        s.started_by_us = true;
                        for (Sensor *o : siblings(s))
                            o->started_by_us = true;
                    }
                }
                if (done_step != Step::LIFECYCLE)
                    grace_siblings(s, t, was_stop ? "duplicate_stop_done" : "action_done");
            }

            // 구독 없이 디스커버리 정보만 묻는다. 데이터 경로에는 아무 영향이 없다.
            GraphView g;
            g.node_checked = !s.node_name.empty();
            g.node_count = g.node_checked
                ? static_cast<std::size_t>(std::count(nodes.begin(), nodes.end(), s.node_name))
                : 1;
            g.publishers = count_publishers(s.topic);
            s.graph = g;

            const auto faults = s.health.diagnose_all(t, g);
            const auto in = [](const std::vector<std::string> &list, Fault f) {
                return std::find(list.begin(), list.end(), std::string(to_string(f))) != list.end();
            };
            bool hard = false, slow = false;
            Fault fault = faults.empty() ? Fault::NONE : faults.front();
            for (Fault f : faults) {
                const bool h = in(act_on_faults_, f);
                const bool sl = in(act_on_faults_slow_, f);
                if ((h || sl) && !hard && !slow)
                    fault = f;            // 첫 행동 대상 고장을 대표로
                hard = hard || h;
                slow = slow || sl;
            }
            s.faults = join_faults(faults);
            const bool duplicate = has_fault(faults, Fault::DUPLICATE);

            // 확실한 고장은 fault_hold_sec, 드라이버 자체 보고(ERROR)는 slow_fault_hold_sec,
            // 행동 대상이 아닌 중복은 duplicate_hold_sec 만큼 참는다
            const double hold = hard ? s.ladder.policy().hold_sec
                              : slow ? slow_fault_hold_
                              : duplicate_hold_;
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

            // 중복만 있고(재시작 대상 고장은 없음) duplicate_hold_sec 가 지났으면.
            // 재시작 쿨다운(COOLDOWN)은 재시작 반복을 막는 장치라 "내가 띄운 중복 끄기"에는 적용하지 않는다.
            const bool dup_due = duplicate && !actionable &&
                                 (v == Verdict::ACT || v == Verdict::COOLDOWN);

            if (dup_due) {
                // 그 유닛을 와치독이 띄웠으면 와치독이 띄운 쪽을 끈다. 아니면 보고만.
                if (s.started_by_us && !s.unit.empty() && !domain_acting(s)) {
                    if (dry_run_) {
                        state = "DRY_RUN_WOULD_stop_duplicate";
                    } else {
                        s.fault = fault;
                        start_duplicate_stop(s, t);
                        state = "STOPPING_DUPLICATE";
                    }
                } else {
                    state = "DUPLICATE_NOT_OURS";   // 와치독이 띄운 게 아니라 손대지 않는다
                }
            }
            else if (v == Verdict::ACT && !actionable) {
                state = "MONITOR_ONLY";
            }
            else if (v == Verdict::ACT) {
                const bool gate_bypass =
                    gate_bypass_after_ >= 0.0 &&
                    s.ladder.bad_since() >= 0.0 &&
                    t - s.ladder.bad_since() >= gate_bypass_after_;

                // 같은 유닛을 다른 센서가 이미 재시작 중이면 중복 재시작하지 않고 기다린다.
                const bool shared_acting =
                    s.ladder.current() != Step::LIFECYCLE && domain_acting(s);

                if (cc)                               state = "BLOCKED_COMMON_CAUSE";
                else if (shared_acting)               state = "WAIT_SHARED_ACTION";
                else if (!gate_ok && !gate_bypass)    state = "BLOCKED_GATE";
                else if (dry_run_)                    state = std::string("DRY_RUN_WOULD_") + to_string(s.ladder.current());
                else {
                    s.fault = fault;
                    if (!gate_ok && gate_bypass)
                        event(s, "gate_bypass", "", -1,
                            "fault persisted " + std::to_string(t - s.ladder.bad_since()) + "s");
                    start_action(s, t);
                    if (s.action && s.action->step != Step::LIFECYCLE)
                        grace_siblings(s, t, "action_start");
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

        if (events_.is_open()) {
            events_.flush();
            rotate_csv(events_, events_path_, EVENTS_HEADER, log_max_bytes_);
        }
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
        using diagnostic_msgs::msg::KeyValue;

        DiagnosticArray arr;
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
            kv("faults", s.faults);
            kv("state", s.state);
            kv("publishers", std::to_string(s.graph.publishers));
            kv("node", !s.graph.node_checked ? "unchecked"
                       : s.graph.node_count == 0 ? "missing"
                       : s.graph.node_count == 1 ? "visible"
                       : "duplicate x" + std::to_string(s.graph.node_count));
            kv("started_by_watchdog", s.started_by_us ? "true" : "false");
            if (s.diag_names.empty()) {
                kv("diag", "not configured");
            } else {
                kv("diag_level", std::to_string(s.health.diag_level()));
                kv("diag_age_sec", std::to_string(s.health.diag_age(t)));
                kv("diag_count", std::to_string(s.health.count()));
                kv("diag_message", s.health.diag_message());
            }
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
            KeyValue y;
            y.key = "sensor_topic_subscriptions";
            y.value = "0";
            sum.values.push_back(y);
        }
        arr.status.push_back(sum);

        diag_pub_->publish(arr);
    }

    // ---------------- 멤버
    bool dry_run_ = true;
    std::string log_dir_;
    uint64_t log_max_bytes_ = 0;
    std::string events_path_;
    std::string gate_topic_;
    std::string gate_type_;
    Prefix gate_prefix_ = Prefix::NONE;
    int gate_field_bytes_ = 1;
    int gate_ok_value_ = 1;
    bool gate_reliable_ = true;
    double gate_timeout_ = 1.0;
    double gate_bypass_after_ = -1.0;
    int common_cause_min_ = 3;
    double common_cause_window_ = 3.0;
    double action_timeout_ = 30.0;
    std::vector<std::string> systemctl_cmd_;
    std::vector<std::string> act_on_faults_;
    std::vector<std::string> act_on_faults_slow_;
    double slow_fault_hold_ = 5.0;
    double duplicate_hold_ = 5.0;

    bool gate_value_ = false;
    uint64_t gate_last_value_ = 0;
    double gate_rx_ = -1e9;

    uint64_t tick_count_ = 0;

    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::ofstream events_;

    SdNotify sd_;
    std::unique_ptr<UdpHeartbeat> heartbeat_;

    rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;
    rclcpp::GenericSubscription::SharedPtr gate_sub_;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
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
