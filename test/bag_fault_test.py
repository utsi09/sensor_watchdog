#!/usr/bin/env python3
"""실차 로스백으로 와치독을 통째로 돌려 보는 자동 테스트.

와치독은 센서 토픽을 구독하지 않고 그래프(발행자·노드 유무)와 드라이버 /diagnostics 만 본다.
이 테스트는 백을 재생하면서 센서 토픽 하나를 중계기로 가로채 "발행자가 사라짐", "노드가 사라짐"
같은 고장을 흉내 내고, 가짜 드라이버 diag 를 발행해 "diag ERROR / 보고 끊김"도 흉내 낸다.
진짜 watchdog_node 가 /diagnostics 로 내보내는 상태 변화를 받아서 "기대한 시각 안에 기대한
상태가 됐는가, 다른 센서는 건드리지 않았는가, 센서 토픽 Hz 가 떨어지지 않는가"를 확인한다.
끝나면 PASS/FAIL 과 타임라인을 찍고 실패가 하나라도 있으면 종료 코드 1.

사용 (ROS 2 Humble + 이 패키지가 source 된 셸에서):
  python3 test/bag_fault_test.py <백_폴더>                       # 기본: 모든 시나리오 차례로
  python3 test/bag_fault_test.py <백_폴더> --scenario vanish      # 하나만
  python3 test/bag_fault_test.py <백_폴더> --config config/ipc.yaml --fault-sensor gps

시나리오 (기본 고장 대상 = sensors 목록의 첫 센서, --at 초 뒤에 주입):
  healthy       고장 없이 재생. 전부 HEALTHY 가 되고 끝까지 유지돼야 한다 (오탐 검사)
  vanish        발행자 자체가 사라짐               → NO_PUBLISHER 로 행동
  node_gone     드라이버 노드가 사라짐 (node_name 검사) → NODE_NOT_VISIBLE 로 행동
  flap          발행자가 0.5초 사라졌다 돌아옴      → fault_hold_sec 안이라 행동하면 안 됨 (디스커버리 흔들림 오탐 검사)
  stall_blind   발행자는 살아 있고 데이터만 멈춤    → diag 가 없으면 못 잡는다 (설계상 한계를 고정하는 테스트: HEALTHY 유지)
  diag_error    가짜 드라이버 diag 가 ERROR 로 바뀜  → DIAG_ERROR 로 행동 (slow hold 뒤)
  diag_stale    가짜 드라이버 diag 보고가 끊김      → DIAG_STALE 로 행동
  diag_warn     가짜 드라이버 diag 가 WARN 만 냄     → 고장 아님. HEALTHY 유지 (오탐 검사)
  recover       발행자 사라짐 → 가짜 systemctl 이 "restart" 를 받으면 발행자를 다시 만듦
                → 행동 1회 뒤 HEALTHY 로 복귀해야 한다 (복구 경로 끝까지)
  all_lidars    같은 유닛의 센서 발행자를 전부 없앰 → 유닛 재시작은 정확히 1회, 전부 복귀
  duplicate     recover 와 같지만 재시작 뒤 발행자가 둘 생김 (옛 프로세스가 안 죽은 상황)
                → 와치독이 띄운 유닛만 "stop" 1회 → 하나 남고 HEALTHY. restart 는 1회뿐
  duplicate_foreign  와치독이 띄운 적 없는데 발행자가 둘 → DUPLICATE_NOT_OURS 보고만, systemctl 호출 없음
  hz_impact     같은 구간을 와치독 없이 / 있이 재생하고 독립 구독자가 센서 토픽 수신 Hz 를 잰다
                → Hz 가 떨어지지 않아야 하고(기본 허용 1 %), 센서 토픽 구독자 수가 늘지 않아야 한다

"행동"은 --mode 에 따라 dry_run 의 DRY_RUN_WOULD_* 상태이거나(기본), 가짜 systemctl 호출이다.
recover / all_lidars 는 항상 가짜 systemctl 을 쓴다. 진짜 systemctl 은 절대 부르지 않는다.
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ALL_SCENARIOS = ["healthy", "vanish", "node_gone", "flap", "stall_blind",
                 "diag_error", "diag_stale", "diag_warn",
                 "recover", "all_lidars", "duplicate", "duplicate_foreign", "hz_impact"]
RELAY_NODE = "fault_relay"          # node_gone 시나리오에서 와치독이 감시할 노드 이름
FAKE_DIAG_PREFIX = "fake_driver: "  # 가짜 드라이버 diag 의 status 이름 앞부분

FAKE_SYSTEMCTL = r"""#!/usr/bin/env bash
# 테스트용 가짜 systemctl. 호출을 기록하고, restart 면 중계기에 "다시 띄워라" 표시를 남긴다.
echo "$(date +%T) $*" >> "$FAKE_LOG"
if [ "$1" = "restart" ]; then touch "$FAKE_FLAGDIR/restarted_$2"; fi
if [ "$1" = "stop" ]; then touch "$FAKE_FLAGDIR/stopped_$2"; fi
exit 0
"""


def load_config(path):
    import yaml
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    key = next(iter(doc))
    p = doc[key]["ros__parameters"]
    # 설정 파일 키가 /** (아무 노드) 면 테스트용 이름을 하나 정해 쓴다
    node_name = "watchdog_ipc" if ("*" in key or key.startswith("/")) else key
    return node_name, p


def fmt_t(t):
    return f"{t:+7.1f}s"


# --------------------------------------------------------------------------- 중계기 + 진단 수신
class Harness:
    """하나의 시나리오를 띄우고, 상태 변화를 모으고, 끝에 판정한다."""

    # 중계기 동작: None=중계 없음, vanish=발행자 제거, stall=데이터만 멈춤, flap=잠깐 제거
    MODES = {"healthy": None, "vanish": "vanish", "node_gone": "node_gone", "flap": "flap",
             "stall_blind": "stall", "diag_error": None, "diag_stale": None, "diag_warn": None,
             "recover": "vanish", "all_lidars": "vanish",
             "duplicate": "vanish", "duplicate_foreign": "dup", "hz_impact": None}
    DIAG_SCENARIOS = ("diag_error", "diag_stale", "diag_warn")

    def __init__(self, args, scenario):
        self.args = args
        self.scenario = scenario
        self.node_name, self.p = load_config(args.config)
        self.sensors = list(self.p["sensors"])
        self.fault_sensor = args.fault_sensor or self.sensors[0]
        if self.fault_sensor not in self.sensors:
            sys.exit(f"--fault-sensor {self.fault_sensor} 는 sensors 목록에 없음: {self.sensors}")
        unit = self.p[self.fault_sensor].get("unit", "")
        self.unit_siblings = [s for s in self.sensors if unit and self.p[s].get("unit") == unit]
        self.relay_sensors = (self.unit_siblings if scenario == "all_lidars" else [self.fault_sensor])
        self.mode = self.MODES[scenario]
        self.use_fake = scenario in ("recover", "all_lidars", "duplicate", "duplicate_foreign") or args.mode == "fake"
        self.tmp = tempfile.mkdtemp(prefix=f"wdbag_{scenario}_")
        self.events = []            # (t, sensor, state, faults). 판정 전에 t0 기준 상대시각으로 바꾼다
        self.last_state = {}
        self.last_kv = {}           # sensor -> 마지막 진단 값
        self.t0 = None              # 첫 센서 메시지 수신 시각 (monotonic). 타임라인의 0초
        self.t_start = None         # 재생 시작 시각 (t0 를 못 잡았을 때 대신 씀)
        self.count = {}
        self.pubs = {}
        self.types = {}
        self.extra_pubs = {}        # duplicate 시나리오: 같은 토픽의 두 번째 발행자
        self.flap_done = set()
        self.lock = threading.Lock()
        self.fail = []
        self.notes = []

    # ---- 프로세스
    def node_cmd(self):
        binpath = self.args.node_bin
        if not binpath:
            prefix = subprocess.run(["ros2", "pkg", "prefix", "sensor_watchdog"],
                                    capture_output=True, text=True).stdout.strip()
            binpath = os.path.join(prefix, "lib", "sensor_watchdog", "watchdog_node")
        cmd = [binpath, "--ros-args", "-r", f"__node:={self.node_name}",
               "--params-file", self.args.config,
               "-p", f"log_dir:={self.tmp}", "-p", 'heartbeat_host:=""',
               "-p", 'gate_topic:=""']        # 게이트는 백 재생에선 항상 허용 (빈 문자열은 "" 로 줘야 파싱됨)
        if self.mode == "node_gone":
            cmd += ["-p", f"{self.fault_sensor}.node_name:=/{RELAY_NODE}"]
        if self.scenario in self.DIAG_SCENARIOS:
            cmd += ["-p", f"{self.fault_sensor}.diag_names:=['{FAKE_DIAG_PREFIX}{self.fault_sensor}']"]
        if self.use_fake:
            fake = os.path.join(self.tmp, "fake_systemctl.sh")
            with open(fake, "w") as f:
                f.write(FAKE_SYSTEMCTL)
            os.chmod(fake, 0o755)
            cmd += ["-p", "dry_run:=false", "-p", f"systemctl_cmd:=['{fake}']"]
        else:
            cmd += ["-p", "dry_run:=true"]
        return cmd

    def play_cmd(self, remap=True):
        topics = [self.p[s]["topic"] for s in self.sensors]
        cmd = ["ros2", "bag", "play", self.args.bag, "--disable-keyboard-controls",
               "--topics", *topics]
        remaps = [f"{self.p[s]['topic']}:=/bagsrc{self.p[s]['topic']}" for s in self.relay_sensors]
        if remaps and self.mode and remap:
            cmd += ["--remap", *remaps]
        return cmd

    # ---- 고장 주입 (중계기)
    def elapsed(self):
        return time.monotonic() - self.t0 if self.t0 else -1.0

    def faulting(self, sensor):
        if self.mode is None or self.t0 is None:
            return False
        el = self.elapsed()
        if el < self.args.at:
            return False
        if self.mode == "flap":
            return el < self.args.at + 0.5          # 0.5초만 사라졌다 돌아온다
        if self.mode == "dup":
            return False                            # 데이터는 계속 흘리고 발행자만 하나 더 만든다
        if self.use_fake and os.path.exists(
                os.path.join(self.tmp, "restarted_" + self.p[sensor].get("unit", ""))):
            return False
        return True

    def on_sensor_msg(self, sensor, buf):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.count[sensor] = self.count.get(sensor, 0) + 1
        b = bytes(buf)
        if not self.faulting(sensor):
            pub = self.pubs.get(sensor)
            if pub is None:        # 복구: 발행자를 다시 만든다 (드라이버가 다시 뜬 것처럼)
                pub = self.relay.create_publisher(self.types[sensor], self.p[sensor]["topic"], self.pub_qos)
                self.pubs[sensor] = pub
            pub.publish(b)
            self.maybe_duplicate(sensor)
            return
        m = self.mode
        if m == "node_gone":
            return                      # 주 루프가 노드를 통째로 없앤다
        if m in ("vanish", "flap"):
            pub = self.pubs.pop(sensor, None)
            if pub is not None:
                self.relay.destroy_publisher(pub)   # 발행자가 그래프에서 사라진다
            return
        if m == "stall":
            return                      # 발행자는 그대로 두고 데이터만 안 보낸다

    def maybe_duplicate(self, sensor):
        """duplicate: 와치독이 restart 한 뒤 발행자를 하나 더 만든다 (옛 프로세스가 안 죽은 상황).
        와치독이 그 유닛을 stop 하면(stopped_ 플래그) 그때 와치독이 띄운 쪽 = 두 번째 발행자를 없앤다.
        duplicate_foreign: 와치독과 무관하게 --at 부터 발행자가 둘이다. 절대 없어지면 안 된다."""
        unit = self.p[sensor].get("unit", "")
        if self.scenario == "duplicate":
            restarted = os.path.exists(os.path.join(self.tmp, "restarted_" + unit))
            stopped = os.path.exists(os.path.join(self.tmp, "stopped_" + unit))
            if restarted and not stopped and sensor not in self.extra_pubs:
                self.extra_pubs[sensor] = self.relay.create_publisher(
                    self.types[sensor], self.p[sensor]["topic"], self.pub_qos)
            elif stopped and sensor in self.extra_pubs:
                self.relay.destroy_publisher(self.extra_pubs.pop(sensor))
        elif self.scenario == "duplicate_foreign":
            if self.elapsed() >= self.args.at and sensor not in self.extra_pubs:
                self.extra_pubs[sensor] = self.relay.create_publisher(
                    self.types[sensor], self.p[sensor]["topic"], self.pub_qos)

    # ---- 가짜 드라이버 diag (diag_* 시나리오)
    def fake_diag_tick(self):
        if self.t0 is None:
            return
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        el = self.elapsed()
        after = el >= self.args.at
        if self.scenario == "diag_stale" and after:
            return                      # 보고 끊김
        level = DiagnosticStatus.OK
        msg = "ok"
        if after and self.scenario == "diag_error":
            level, msg = DiagnosticStatus.ERROR, "no events in window"
        elif after and self.scenario == "diag_warn":
            level, msg = DiagnosticStatus.WARN, "rate slightly low"
        arr = DiagnosticArray()
        arr.header.stamp = self.diag_node.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = FAKE_DIAG_PREFIX + self.fault_sensor
        st.level = level
        st.message = msg
        st.hardware_id = self.p[self.fault_sensor]["topic"]
        arr.status.append(st)
        other = DiagnosticStatus()
        other.name = "unrelated: something"
        other.level = DiagnosticStatus.ERROR
        other.message = "must be ignored"
        arr.status.append(other)
        self.fake_diag_pub.publish(arr)

    # ---- 와치독 진단 수신
    def on_diag(self, msg):
        prefix = self.node_name + ": "
        now = time.monotonic()
        for st in msg.status:
            if not st.name.startswith(prefix):
                continue
            sensor = st.name[len(prefix):]
            if sensor == "summary" or sensor not in self.sensors:
                continue
            kv = {x.key: x.value for x in st.values}
            state, faults = kv.get("state", "?"), kv.get("faults", "")
            with self.lock:
                self.last_kv[sensor] = kv
                if self.last_state.get(sensor) != (state, faults):
                    self.last_state[sensor] = (state, faults)
                    self.events.append((now, sensor, state, faults))

    # ---- 실행
    def run(self):
        if self.scenario == "hz_impact":
            return self.run_hz_impact()
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.executors import MultiThreadedExecutor
        from diagnostic_msgs.msg import DiagnosticArray

        env = dict(os.environ, FAKE_LOG=os.path.join(self.tmp, "systemctl.log"),
                   FAKE_FLAGDIR=self.tmp)
        if self.args.domain is not None:
            env["ROS_DOMAIN_ID"] = str(self.args.domain)
        rclpy.init(domain_id=self.args.domain)
        node = rclpy.create_node("bag_fault_test")
        self.diag_node = node
        self.relay = rclpy.create_node(RELAY_NODE)
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        if self.mode:
            for s in self.relay_sensors:
                T = self.topic_type(s)
                self.types[s] = T
                self.pubs[s] = self.relay.create_publisher(T, self.p[s]["topic"], self.pub_qos)
                self.relay.create_subscription(T, "/bagsrc" + self.p[s]["topic"],
                                               lambda b, s=s: self.on_sensor_msg(s, b),
                                               sub_qos, raw=True)
        else:
            # 중계 없이도 "첫 메시지" 시각을 잡기 위해 가볍게 하나만 구독한다 (와치독이 아니라 테스트의 구독)
            s = self.fault_sensor
            T = self.topic_type(s)
            self.relay.create_subscription(T, self.p[s]["topic"],
                                           lambda b, s=s: self.on_first_msg(), sub_qos, raw=True)
        node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        if self.scenario in self.DIAG_SCENARIOS:
            self.fake_diag_pub = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
            node.create_timer(1.0, self.fake_diag_tick)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.add_node(self.relay)
        spinner = threading.Thread(target=ex.spin, daemon=True)
        spinner.start()

        wd_log = open(os.path.join(self.tmp, "watchdog.log"), "w")
        wd = subprocess.Popen(self.node_cmd(), env=env, stdout=wd_log, stderr=subprocess.STDOUT)
        time.sleep(2.0)
        play_log = open(os.path.join(self.tmp, "play.log"), "w")
        play = subprocess.Popen(self.play_cmd(), env=env, stdout=play_log, stderr=subprocess.STDOUT)
        self.t_start = time.monotonic()
        print(f"[{self.scenario}] 와치독 pid={wd.pid}, 재생 pid={play.pid}, 로그 {self.tmp}")
        relay_alive = True
        deadline = time.monotonic() + self.args.duration + 5.0
        try:
            while time.monotonic() < deadline:
                if play.poll() is not None or wd.poll() is not None:
                    break
                el = self.elapsed()
                if el >= self.args.duration:
                    break
                if self.mode == "node_gone" and relay_alive and el >= self.args.at:
                    ex.remove_node(self.relay)      # 드라이버 노드가 죽은 것처럼 그래프에서 사라진다
                    self.relay.destroy_node()
                    relay_alive = False
                time.sleep(0.2)
        finally:
            for pr in (play, wd):
                if pr.poll() is None:
                    pr.send_signal(signal.SIGINT)
            for pr in (play, wd):
                try:
                    pr.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pr.kill()
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
            if relay_alive:
                self.relay.destroy_node()
            rclpy.shutdown()
            wd_log.close(); play_log.close()
        if wd.returncode not in (0, -2, None) and wd.returncode != 130:
            self.fail.append(f"watchdog_node 가 비정상 종료 (코드 {wd.returncode}) — {self.tmp}/watchdog.log")
        if self.t0 is None:
            self.fail.append("센서 메시지를 하나도 못 받음 (백 토픽 이름, 도메인 확인)")
            self.t0 = self.t_start
        with self.lock:
            self.events = [(t - self.t0, s, st, f) for t, s, st, f in self.events]
        return self.judge()

    def on_first_msg(self):
        if self.t0 is None:
            self.t0 = time.monotonic()

    def topic_type(self, sensor):
        """설정에 type 이 없으면 백 metadata 에서 토픽 타입을 찾는다."""
        from rosidl_runtime_py.utilities import get_message
        t = self.p[sensor].get("type")
        if not t:
            import yaml
            with open(os.path.join(self.args.bag, "metadata.yaml"), encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            for tm in meta["rosbag2_bagfile_information"]["topics_with_message_count"]:
                if tm["topic_metadata"]["name"] == self.p[sensor]["topic"]:
                    t = tm["topic_metadata"]["type"]
                    break
        if not t:
            sys.exit(f"{sensor}: 토픽 타입을 알 수 없음 (설정 type 또는 백 metadata)")
        return get_message(t)

    # ---- Hz 영향 측정: 같은 구간을 와치독 없이(A) / 있이(B) 재생하고 독립 구독자가 Hz 를 잰다
    def run_hz_impact(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.executors import MultiThreadedExecutor
        from diagnostic_msgs.msg import DiagnosticArray

        env = dict(os.environ)
        if self.args.domain is not None:
            env["ROS_DOMAIN_ID"] = str(self.args.domain)
        rclpy.init(domain_id=self.args.domain)
        node = rclpy.create_node("hz_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        arrivals = {s: [] for s in self.sensors}
        for s in self.sensors:
            node.create_subscription(self.topic_type(s), self.p[s]["topic"],
                                     lambda b, s=s: arrivals[s].append(time.monotonic()), qos, raw=True)
        node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        sub_counts = {}

        def measure(with_watchdog):
            for s in self.sensors:
                arrivals[s].clear()
            wd = None
            if with_watchdog:
                wd_log = open(os.path.join(self.tmp, "watchdog.log"), "w")
                wd = subprocess.Popen(self.node_cmd(), env=env, stdout=wd_log, stderr=subprocess.STDOUT)
                time.sleep(2.0)
            play_log = open(os.path.join(self.tmp, f"play_{'B' if with_watchdog else 'A'}.log"), "w")
            play = subprocess.Popen(self.play_cmd(remap=False), env=env, stdout=play_log,
                                    stderr=subprocess.STDOUT)
            t_end = time.monotonic() + self.args.duration
            counted = False
            while time.monotonic() < t_end and play.poll() is None:
                if not counted and time.monotonic() > t_end - self.args.duration + 8.0:
                    # 재생 8초 뒤 (디스커버리 끝난 뒤) 센서 토픽 구독자 수를 센다
                    sub_counts["B" if with_watchdog else "A"] = {
                        s: node.count_subscribers(self.p[s]["topic"]) for s in self.sensors}
                    counted = True
                time.sleep(0.2)
            for pr in (play, wd):
                if pr is not None and pr.poll() is None:
                    pr.send_signal(signal.SIGINT)
            for pr in (play, wd):
                if pr is not None:
                    try:
                        pr.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        pr.kill()
            play_log.close()
            out = {}
            for s in self.sensors:
                a = arrivals[s]
                # 처음 2초(재생 램프업)와 마지막 0.5초는 버리고 잰다
                if len(a) < 10:
                    out[s] = (0.0, len(a), 0.0)
                    continue
                lo, hi = a[0] + 2.0, a[-1] - 0.5
                w = [t for t in a if lo <= t <= hi]
                gaps = [y - x for x, y in zip(w, w[1:])]
                out[s] = ((len(w) - 1) / max(w[-1] - w[0], 1e-9), len(w), max(gaps) if gaps else 0.0)
            return out

        print(f"[hz_impact] A: 와치독 없이 {self.args.duration:.0f}s 재생")
        A = measure(False)
        time.sleep(1.0)
        print(f"[hz_impact] B: 와치독 켜고 {self.args.duration:.0f}s 재생  (로그 {self.tmp})")
        B = measure(True)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()

        tol = self.args.hz_tolerance
        print(f"\n--- [hz_impact] 독립 구독자가 잰 수신 Hz (A 없이 / B 있이, 허용 저하 {tol * 100:.0f} %) ---")
        print(f"  {'센서':<10} {'A Hz':>7} {'B Hz':>7} {'비율':>6} {'최대간격A':>9} {'최대간격B':>9}  구독자수 A→B")
        for s in self.sensors:
            a_hz, a_n, a_gap = A[s]
            b_hz, b_n, b_gap = B[s]
            ratio = b_hz / a_hz if a_hz > 0 else 0.0
            ca = sub_counts.get("A", {}).get(s, "?")
            cb = sub_counts.get("B", {}).get(s, "?")
            print(f"  {s:<10} {a_hz:7.2f} {b_hz:7.2f} {ratio:6.3f} {a_gap * 1e3:8.0f}ms {b_gap * 1e3:8.0f}ms  {ca}→{cb}")
            self.expect(a_n >= 10 and b_n >= 10, f"{s}: 수신 메시지가 너무 적음 (A {a_n}, B {b_n})")
            self.expect(ratio >= 1.0 - tol, f"{s}: 와치독을 켜니 수신 Hz 가 {a_hz:.2f} → {b_hz:.2f} 로 떨어짐")
            self.expect(ca == cb, f"{s}: 와치독을 켜니 센서 토픽 구독자 수가 {ca} → {cb} 로 늘었다 (구독 금지 위반)")
        return self.report(timeline=False)

    # ---- 판정
    def first(self, sensor, pred, after=-1e9):
        for t, s, state, faults in self.events:
            if s == sensor and t >= after and pred(state, faults):
                return t, state, faults
        return None

    def states_after(self, sensor, after):
        return [(t, st, f) for t, s, st, f in self.events if s == sensor and t >= after]

    def expect(self, cond, msg):
        if not cond:
            self.fail.append(msg)

    def judge(self):
        p, at = self.p, self.args.at
        grace = float(p.get("startup_grace_sec", 10.0))
        hold = float(p.get("fault_hold_sec", 2.0))
        slow_hold = float(p.get("slow_fault_hold_sec", 5.0))
        fs = self.fault_sensor
        diag_timeout = float(p[fs].get("diag_timeout_sec", 3.0))
        others = [s for s in self.sensors if s not in self.relay_sensors]
        act = (lambda st, f: st.startswith("DRY_RUN_WOULD") or st in ("ACTING", "WAIT_SHARED_ACTION"))
        is_bad = (lambda st, f: st not in ("GRACE", "HEALTHY"))

        # 공통: 모든 센서가 유예 뒤 HEALTHY
        for s in self.sensors:
            h = self.first(s, lambda st, f: st == "HEALTHY")
            self.expect(h is not None and h[0] <= grace + 8.0,
                        f"{s}: HEALTHY 가 {grace + 8:.0f}s 안에 안 됨 (첫 HEALTHY: {h})")
        pre_limit = at - 0.5
        for s in self.sensors:
            bad = [e for e in self.states_after(s, 0.0) if e[0] < pre_limit and is_bad(e[1], e[2])]
            self.expect(not bad, f"{s}: 고장 주입 전에 비정상 상태 {bad[:2]}")

        sc = self.scenario
        if sc in ("healthy", "stall_blind", "diag_warn", "flap"):
            why = {"healthy": "고장이 없는데",
                   "stall_blind": "발행자가 살아 있는 데이터 정지는 diag 없이 못 잡는 게 설계인데",
                   "diag_warn": "diag 가 WARN 뿐인데",
                   "flap": "발행자가 0.5초만 사라졌는데 (fault_hold_sec 안)"}[sc]
            for s in self.sensors:
                bad = [e for e in self.states_after(s, grace + 8.0)
                       if is_bad(e[1], e[2]) and not (sc == "flap" and e[1] == "HOLDING")]
                self.expect(not bad, f"{s}: {why} 비정상 상태 (오탐) {bad[:3]}")
            if sc == "stall_blind":
                self.notes.append(f"{fs}: 발행자 유지 + 데이터 정지 → 상태 변화 없음. "
                                  f"설계상 한계 (잡으려면 드라이버 /diagnostics 필요 → diag_names)")
            if sc == "diag_warn":
                lv = self.last_kv.get(fs, {}).get("diag_level", "?")
                self.notes.append(f"{fs}: 와치독이 본 diag_level={lv} (WARN=1 은 기본 설정에서 고장 아님)")
                self.expect(lv == "1", f"{fs}: 가짜 diag WARN 이 와치독에 전달되지 않음 (diag_level {lv})")
            if sc == "flap":
                hold_seen = self.first(fs, lambda st, f: st == "HOLDING", after=at)
                self.notes.append(f"{fs}: 0.5초 발행자 소실 → {'HOLDING 만 거치고' if hold_seen else '상태 변화 없이'} 복귀, 행동 없음")
        elif sc in ("vanish", "node_gone", "diag_error", "diag_stale"):
            fault, budget = {
                "vanish":     ("NO_PUBLISHER", hold + 3.0),
                "node_gone":  ("NODE_NOT_VISIBLE", hold + 3.0),
                "diag_error": ("DIAG_ERROR", slow_hold + 3.0),          # 1 Hz 보고 + 틱
                "diag_stale": ("DIAG_STALE", diag_timeout + hold + 3.0),
            }[sc]
            hit = self.first(fs, lambda st, f: act(st, f) and fault in f, after=at)
            self.expect(hit is not None, f"{fs}: 주입 뒤 '{fault}' 로 행동 상태가 안 됨. 기록: {self.states_after(fs, at)[:4]}")
            if hit:
                self.expect(hit[0] - at <= budget,
                            f"{fs}: 행동까지 {hit[0] - at:.1f}s 걸림 (허용 {budget:.1f}s)")
                self.notes.append(f"{fs}: 주입 +{hit[0] - at:.1f}s 에 {hit[1]} ({hit[2]})")
        elif sc in ("recover", "all_lidars"):
            calls = self.systemctl_calls()
            restarts = [c for c in calls if c.startswith("restart ")]
            self.expect(len(restarts) == 1, f"유닛 재시작 호출이 1회여야 하는데 {len(restarts)}회: {calls}")
            for s in self.relay_sensors:
                # 가짜 systemctl 은 즉시 끝나서 ACTING 이 0.2초 진단 주기보다 짧을 수 있다.
                # 그래서 "행동 뒤 유예(GRACE)" 까지 행동의 증거로 본다.
                acted = self.first(s, lambda st, f: st in ("ACTING", "WAIT_SHARED_ACTION", "GRACE"), after=at)
                self.expect(acted is not None, f"{s}: 행동/대기/유예 상태가 안 됨. 기록: {self.states_after(s, at)[:4]}")
                back = self.first(s, lambda st, f: st == "HEALTHY", after=(acted[0] if acted else at))
                self.expect(back is not None and back[0] - at <= grace + hold + 10.0,
                            f"{s}: 재시작 뒤 HEALTHY 복귀 실패/지연 (복귀: {back})")
                if acted and back:
                    self.notes.append(f"{s}: 주입 +{acted[0] - at:.1f}s {acted[1]} → +{back[0] - at:.1f}s HEALTHY")
            if sc == "all_lidars":
                blocked = [s for s in self.relay_sensors
                           if self.first(s, lambda st, f: st == "BLOCKED_COMMON_CAUSE", after=at)]
                self.expect(not blocked, f"같은 유닛인데 공통 원인으로 막힘: {blocked}")
            # 재시작한 유닛은 started_units 에 적혀야 한다 (watchdog_off.sh --all 이 읽는 파일)
            units = self.started_units()
            self.expect(p[fs]["unit"] in units,
                        f"started_units 에 {p[fs]['unit']} 가 없음: {units}")
            self.expect(len(units) == 1, f"started_units 는 유닛 1개여야 하는데: {units}")

        elif sc == "duplicate":
            calls = self.systemctl_calls()
            self.expect(calls == [f"restart {p[fs]['unit']}", f"stop {p[fs]['unit']}"],
                        f"호출이 [restart, stop] 1회씩이어야 하는데: {calls}")
            dup = self.first(fs, lambda st, f: "DUPLICATE" in f, after=at)
            self.expect(dup is not None, f"{fs}: 재시작 뒤 DUPLICATE 가 보고되지 않음. 기록: {self.states_after(fs, at)[:6]}")
            stopping = self.first(fs, lambda st, f: st in ("STOPPING_DUPLICATE", "GRACE"), after=(dup[0] if dup else at))
            back = self.first(fs, lambda st, f: st == "HEALTHY", after=(stopping[0] if stopping else at))
            self.expect(back is not None, f"{fs}: 중복 정리 뒤 HEALTHY 복귀 실패. 기록: {self.states_after(fs, at)[-6:]}")
            if dup and back:
                self.notes.append(f"{fs}: 재시작 후 발행자 2개 → +{dup[0] - at:.1f}s DUPLICATE 감지 → stop 호출 → +{back[0] - at:.1f}s HEALTHY (발행자 1개)")
            self.expect(not self.started_units(),
                        f"와치독이 띄운 유닛을 stop 한 뒤엔 started_units 가 비어야 하는데: {self.started_units()}")
            self.expect(not [e for e in self.states_after(fs, at) if e[1] == "DUPLICATE_NOT_OURS"],
                        f"{fs}: 와치독이 띄운 유닛인데 DUPLICATE_NOT_OURS 로 분류됨")
        elif sc == "duplicate_foreign":
            calls = self.systemctl_calls()
            self.expect(not calls, f"와치독이 띄운 게 아닌데 systemctl 을 호출함: {calls}")
            seen = self.first(fs, lambda st, f: st == "DUPLICATE_NOT_OURS" and "DUPLICATE" in f, after=at)
            self.expect(seen is not None, f"{fs}: DUPLICATE_NOT_OURS 가 보고되지 않음. 기록: {self.states_after(fs, at)[:6]}")
            if seen:
                self.notes.append(f"{fs}: 남이 띄운 중복 → +{seen[0] - at:.1f}s DUPLICATE_NOT_OURS 보고만, 호출 없음")

        # 공통: 고장 대상이 아닌 센서는 끝까지 HEALTHY
        for s in others:
            bad = [e for e in self.states_after(s, at) if is_bad(e[1], e[2])]
            self.expect(not bad, f"{s}: 고장 대상이 아닌데 상태가 바뀜 (부수 피해) {bad[:3]}")
        if not self.use_fake:
            self.expect(not self.systemctl_calls(), "dry_run 인데 systemctl 이 호출됨")
        return self.report()

    def started_units(self):
        """와치독이 log_dir 에 적는 started_units 파일의 유닛 이름 목록 (없으면 [])."""
        path = os.path.join(self.tmp, "started_units")
        if not os.path.exists(path):
            return []
        with open(path, encoding="utf-8") as f:
            return [line.split("\t")[0] for line in f.read().splitlines() if line.strip()]

    def systemctl_calls(self):
        path = os.path.join(self.tmp, "systemctl.log")
        if not os.path.exists(path):
            return []
        with open(path) as f:
            return [" ".join(line.split()[1:]) for line in f if line.strip()]

    def report(self, timeline=True):
        if timeline:
            print(f"\n--- [{self.scenario}] 타임라인 (0 = 첫 센서 메시지, 고장 주입 {self.args.at:.0f}s) ---")
            for t, s, state, faults in self.events:
                print(f"  {fmt_t(t)}  {s:<10} {state}{'  ' + faults if faults and faults != 'NONE' else ''}")
            calls = self.systemctl_calls()
            if calls:
                print("  systemctl 호출:", calls)
        for n in self.notes:
            print("  *", n)
        ok = not self.fail
        print(f"[{self.scenario}] {'PASS' if ok else 'FAIL'}")
        for f in self.fail:
            print("   -", f)
        if ok and not self.args.keep:
            shutil.rmtree(self.tmp, ignore_errors=True)
        else:
            print(f"   로그: {self.tmp}")
        return ok


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="ros2 bag 폴더")
    ap.add_argument("--config", default=os.path.join(PKG, "config", "ipc.yaml"))
    ap.add_argument("--scenario", default="all", help="all 또는 쉼표로 이어 쓴 이름: " + ",".join(ALL_SCENARIOS))
    ap.add_argument("--fault-sensor", default=None, help="고장을 넣을 센서 id (기본: sensors 첫 항목)")
    ap.add_argument("--at", type=float, default=15.0, help="첫 메시지 기준 고장 주입 시각 [s]")
    ap.add_argument("--duration", type=float, default=45.0, help="시나리오당 재생 시간 [s]")
    ap.add_argument("--mode", choices=["dry", "fake"], default="dry",
                    help="dry: dry_run 상태만 확인 / fake: 가짜 systemctl 로 행동까지")
    ap.add_argument("--hz-tolerance", type=float, default=0.01, help="hz_impact 허용 저하 비율 (기본 1 %%)")
    ap.add_argument("--domain", type=int, default=86, help="ROS_DOMAIN_ID (다른 노드와 분리)")
    ap.add_argument("--node-bin", default=None, help="watchdog_node 경로 (기본: ros2 pkg prefix 로 찾음)")
    ap.add_argument("--keep", action="store_true", help="PASS 해도 로그 폴더를 남긴다")
    args = ap.parse_args()
    if not os.path.isdir(args.bag):
        sys.exit(f"백 폴더가 없음: {args.bag}")
    try:
        import rclpy  # noqa: F401
    except ImportError:
        sys.exit("ROS 2 환경이 필요합니다 (source /opt/ros/humble/setup.bash && source install/setup.bash)")

    scenarios = ALL_SCENARIOS if args.scenario == "all" else args.scenario.split(",")
    bad = [s for s in scenarios if s not in ALL_SCENARIOS]
    if bad:
        sys.exit(f"모르는 시나리오: {bad}")
    if len(scenarios) == 1:
        ok = Harness(args, scenarios[0]).run()
        sys.exit(0 if ok else 1)

    # 여러 개는 시나리오마다 새 프로세스로 (rclpy 재초기화 문제 회피)
    results = {}
    for sc in scenarios:
        cmd = [sys.executable, os.path.abspath(__file__), args.bag, "--scenario", sc,
               "--config", args.config, "--at", str(args.at), "--duration", str(args.duration),
               "--mode", args.mode, "--domain", str(args.domain),
               "--hz-tolerance", str(args.hz_tolerance)]
        if args.fault_sensor:
            cmd += ["--fault-sensor", args.fault_sensor]
        if args.node_bin:
            cmd += ["--node-bin", args.node_bin]
        if args.keep:
            cmd += ["--keep"]
        results[sc] = subprocess.run(cmd).returncode == 0
        time.sleep(1.0)
    print("\n=== 요약 ===")
    for sc, ok in results.items():
        print(f"  {sc:<11} {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
