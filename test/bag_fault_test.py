#!/usr/bin/env python3
"""실차 로스백으로 와치독을 통째로 돌려 보는 자동 테스트.

백을 재생하면서 센서 토픽 하나를 중간에서 가로채 고장을 흉내 내고(끊김·멈춘 stamp·주기 저하 등),
진짜 watchdog_node 가 /diagnostics 로 내보내는 상태 변화를 받아서 "기대한 시각 안에 기대한
상태가 됐는가, 다른 센서는 건드리지 않았는가"를 확인한다. 끝나면 PASS/FAIL 과 타임라인을 찍고
실패가 하나라도 있으면 종료 코드 1.

사용 (ROS 2 Humble + 이 패키지가 source 된 셸에서):
  python3 test/bag_fault_test.py <백_폴더>                       # 기본: 모든 시나리오 차례로
  python3 test/bag_fault_test.py <백_폴더> --scenario stop        # 하나만
  python3 test/bag_fault_test.py <백_폴더> --config config/ipc.yaml --fault-sensor gps

시나리오 (기본 고장 대상 = sensors 목록의 첫 센서, --at 초 뒤에 주입):
  healthy     고장 없이 재생. 전부 HEALTHY 가 되고 끝까지 유지돼야 한다 (오탐 검사)
  stop        토픽 끊김            → RX_TIMEOUT 으로 행동. 나머지 센서는 HEALTHY 유지
  freeze      stamp 가 멈춘 채 발행 → STAMP_NOT_ADVANCING 으로 행동
  slow        3개 중 1개만 통과    → LOW_RX_HZ 로 행동
  zero_stamp  stamp 0             → STAMP_INVALID_OR_BACKWARD 로 행동
  truncate    메시지를 2 KB 로 자름 → TOO_SMALL 은 기록만 (MONITOR_ONLY), 행동 없음
  recover     끊김 → 가짜 systemctl 이 "restart" 를 받으면 토픽을 다시 흘려 줌
              → 행동 1회 뒤 HEALTHY 로 복귀해야 한다 (복구 경로 끝까지)
  all_lidars  같은 유닛의 센서를 전부 끊음 → 유닛 재시작은 정확히 1회, 전부 복귀

"행동"은 --mode 에 따라 dry_run 의 DRY_RUN_WOULD_* 상태이거나(기본), 가짜 systemctl 호출이다.
recover / all_lidars 는 항상 가짜 systemctl 을 쓴다. 진짜 systemctl 은 절대 부르지 않는다.
"""
import argparse
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ALL_SCENARIOS = ["healthy", "stop", "freeze", "slow", "zero_stamp", "truncate", "recover", "all_lidars"]

FAKE_SYSTEMCTL = r"""#!/usr/bin/env bash
# 테스트용 가짜 systemctl. 호출을 기록하고, restart 면 중계기에 "다시 흘려라" 표시를 남긴다.
echo "$(date +%T) $*" >> "$FAKE_LOG"
if [ "$1" = "restart" ]; then touch "$FAKE_FLAGDIR/restarted_$2"; fi
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
        self.mode = {"healthy": None, "stop": "stop", "freeze": "freeze", "slow": "slow",
                     "zero_stamp": "zero_stamp", "truncate": "truncate",
                     "recover": "stop", "all_lidars": "stop"}[scenario]
        self.use_fake = scenario in ("recover", "all_lidars") or args.mode == "fake"
        self.tmp = tempfile.mkdtemp(prefix=f"wdbag_{scenario}_")
        self.events = []            # (rel_t, sensor, state, faults)
        self.last_state = {}
        self.t0 = None              # 첫 센서 메시지 수신 시각 (monotonic). 타임라인의 0초
        self.t_start = None         # 재생 시작 시각 (t0 를 못 잡았을 때 대신 씀)
        self.frozen = {}
        self.count = {}
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
               "-p", f"raw_log_dir:={self.tmp}", "-p", 'heartbeat_host:=""',
               "-p", 'gate_topic:=""']        # 게이트는 백 재생에선 항상 허용 (빈 문자열은 "" 로 줘야 파싱됨)
        if self.use_fake:
            fake = os.path.join(self.tmp, "fake_systemctl.sh")
            with open(fake, "w") as f:
                f.write(FAKE_SYSTEMCTL)
            os.chmod(fake, 0o755)
            cmd += ["-p", "dry_run:=false", "-p", f"systemctl_cmd:=['{fake}']"]
        else:
            cmd += ["-p", "dry_run:=true"]
        return cmd

    def play_cmd(self):
        topics = [self.p[s]["topic"] for s in self.sensors]
        cmd = ["ros2", "bag", "play", self.args.bag, "--disable-keyboard-controls",
               "--topics", *topics]
        remaps = [f"{self.p[s]['topic']}:=/bagsrc{self.p[s]['topic']}" for s in self.relay_sensors]
        if remaps and self.mode:
            cmd += ["--remap", *remaps]
        return cmd

    # ---- 고장 주입
    def faulting(self, sensor):
        if self.mode is None or self.t0 is None:
            return False
        if time.monotonic() - self.t0 < self.args.at:
            return False
        if self.use_fake and os.path.exists(
                os.path.join(self.tmp, "restarted_" + self.p[sensor].get("unit", ""))):
            return False
        return True

    def on_sensor_msg(self, sensor, buf, pub):
        if self.t0 is None:
            self.t0 = time.monotonic()
        self.count[sensor] = self.count.get(sensor, 0) + 1
        b = bytes(buf)
        if not self.faulting(sensor):
            self.frozen.pop(sensor, None)
            pub.publish(b)
            return
        m = self.mode
        if m == "stop":
            return
        if m == "freeze":
            self.frozen.setdefault(sensor, b[4:12])
            pub.publish(b[:4] + self.frozen[sensor] + b[12:])
        elif m == "slow":
            if self.count[sensor] % 3 == 0:
                pub.publish(b)
        elif m == "zero_stamp":
            pub.publish(b[:4] + struct.pack("<iI", 0, 0) + b[12:])
        elif m == "truncate":
            pub.publish(b[:2000])

    # ---- 진단 수신
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
                if self.last_state.get(sensor) != (state, faults):
                    self.last_state[sensor] = (state, faults)
                    self.events.append((now, sensor, state, faults))   # 절대 시각. 판정 때 t0 기준으로 바꾼다

    # ---- 실행
    def run(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.executors import MultiThreadedExecutor
        from rosidl_runtime_py.utilities import get_message
        from diagnostic_msgs.msg import DiagnosticArray

        env = dict(os.environ, FAKE_LOG=os.path.join(self.tmp, "systemctl.log"),
                   FAKE_FLAGDIR=self.tmp)
        if self.args.domain is not None:
            env["ROS_DOMAIN_ID"] = str(self.args.domain)
        rclpy.init(domain_id=self.args.domain)
        node = rclpy.create_node("bag_fault_test")
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        if self.mode:
            for s in self.relay_sensors:
                T = get_message(self.p[s]["type"])
                pub = node.create_publisher(T, self.p[s]["topic"], pub_qos)
                node.create_subscription(T, "/bagsrc" + self.p[s]["topic"],
                                         lambda b, s=s, pub=pub: self.on_sensor_msg(s, b, pub),
                                         sub_qos, raw=True)
        node.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        spinner = threading.Thread(target=ex.spin, daemon=True)
        spinner.start()

        wd_log = open(os.path.join(self.tmp, "watchdog.log"), "w")
        wd = subprocess.Popen(self.node_cmd(), env=env, stdout=wd_log, stderr=subprocess.STDOUT)
        time.sleep(2.0)
        play_log = open(os.path.join(self.tmp, "play.log"), "w")
        play = subprocess.Popen(self.play_cmd(), env=env, stdout=play_log, stderr=subprocess.STDOUT)
        self.t_start = time.monotonic()
        print(f"[{self.scenario}] 와치독 pid={wd.pid}, 재생 pid={play.pid}, 로그 {self.tmp}")
        if self.mode is None:
            self.t0 = time.monotonic()         # 중계기가 없으니 재생 시작을 기준으로
        deadline = time.monotonic() + self.args.duration + 5.0
        try:
            while time.monotonic() < deadline:
                if play.poll() is not None or wd.poll() is not None:
                    break
                if self.t0 and time.monotonic() - self.t0 >= self.args.duration:
                    break
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
            rclpy.shutdown()
            wd_log.close(); play_log.close()
        if wd.returncode not in (0, -2, None) and wd.returncode != 130:
            self.fail.append(f"watchdog_node 가 비정상 종료 (코드 {wd.returncode}) — {self.tmp}/watchdog.log")
        if self.t0 is None:
            self.fail.append("중계기가 센서 메시지를 하나도 못 받음 (백 토픽 이름/타입, 도메인 확인)")
            self.t0 = self.t_start
        with self.lock:
            self.events = [(t - self.t0, s, st, f) for t, s, st, f in self.events]
        return self.judge()

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
        if sc == "healthy":
            for s in self.sensors:
                bad = [e for e in self.states_after(s, grace + 8.0) if is_bad(e[1], e[2])]
                self.expect(not bad, f"{s}: 고장이 없는데 비정상 상태 (오탐) {bad[:3]}")
        elif sc in ("stop", "freeze", "slow", "zero_stamp"):
            fault, budget = {
                "stop":       ("RX_TIMEOUT", float(p[fs].get("timeout_sec", 1.0)) + hold + 2.0),
                "freeze":     ("STAMP_NOT_ADVANCING", float(p[fs].get("stamp_timeout_sec", 1.0)) + slow_hold + 2.0),
                "slow":       ("LOW_RX_HZ", float(p[fs].get("hz_window_sec", 5.0)) + slow_hold + 3.0),
                "zero_stamp": ("STAMP_INVALID_OR_BACKWARD", slow_hold + 2.0),
            }[sc]
            hit = self.first(fs, lambda st, f: act(st, f) and fault in f, after=at)
            self.expect(hit is not None, f"{fs}: 주입 뒤 '{fault}' 로 행동 상태가 안 됨. 기록: {self.states_after(fs, at)[:4]}")
            if hit:
                self.expect(hit[0] - at <= budget,
                            f"{fs}: 행동까지 {hit[0] - at:.1f}s 걸림 (허용 {budget:.1f}s)")
                self.notes.append(f"{fs}: 주입 +{hit[0] - at:.1f}s 에 {hit[1]} ({hit[2]})")
        elif sc == "truncate":
            hit = self.first(fs, lambda st, f: "TOO_SMALL" in f, after=at)
            self.expect(hit is not None, f"{fs}: TOO_SMALL 이 보고되지 않음. 기록: {self.states_after(fs, at)[:4]}")
            acted = self.first(fs, act, after=at)
            self.expect(acted is None, f"{fs}: TOO_SMALL 은 기록만 해야 하는데 행동함 {acted}")
            if hit:
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

        # 공통: 고장 대상이 아닌 센서는 끝까지 HEALTHY
        for s in others:
            bad = [e for e in self.states_after(s, at) if is_bad(e[1], e[2])]
            self.expect(not bad, f"{s}: 고장 대상이 아닌데 상태가 바뀜 (부수 피해) {bad[:3]}")
        if self.mode and not self.use_fake:
            self.expect(not self.systemctl_calls(), "dry_run 인데 systemctl 이 호출됨")
        return self.report()

    def systemctl_calls(self):
        path = os.path.join(self.tmp, "systemctl.log")
        if not os.path.exists(path):
            return []
        with open(path) as f:
            return [" ".join(line.split()[1:]) for line in f if line.strip()]

    def report(self):
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
    ap.add_argument("--scenario", default="all", help="all 또는 " + ",".join(ALL_SCENARIOS))
    ap.add_argument("--fault-sensor", default=None, help="고장을 넣을 센서 id (기본: sensors 첫 항목)")
    ap.add_argument("--at", type=float, default=15.0, help="첫 메시지 기준 고장 주입 시각 [s]")
    ap.add_argument("--duration", type=float, default=45.0, help="시나리오당 재생 시간 [s]")
    ap.add_argument("--mode", choices=["dry", "fake"], default="dry",
                    help="dry: dry_run 상태만 확인 / fake: 가짜 systemctl 로 행동까지")
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
    if args.scenario != "all" and len(scenarios) == 1:
        ok = Harness(args, scenarios[0]).run()
        sys.exit(0 if ok else 1)

    # 여러 개는 시나리오마다 새 프로세스로 (rclpy 재초기화 문제 회피)
    results = {}
    for sc in scenarios:
        cmd = [sys.executable, os.path.abspath(__file__), args.bag, "--scenario", sc,
               "--config", args.config, "--at", str(args.at), "--duration", str(args.duration),
               "--mode", args.mode, "--domain", str(args.domain)]
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
