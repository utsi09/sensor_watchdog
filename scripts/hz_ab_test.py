#!/usr/bin/env python3
"""와치독(또는 아무 구독자)이 하나 더 붙으면 토픽 Hz 가 떨어지는지 A/B 로 잰다.

세 역할을 한 파일에 담았다. 터미널 세 개(또는 두 개)에서 각각 띄운다.

  [1] 발행자  : 라이다 크기의 PointCloud2 를 지정 Hz 로 발행하고, publish() 호출이
               얼마나 걸렸는지(막힘 여부) p50/p99/max 를 찍는다.
  [2] 소비자  : 인지 노드 역할. 수신 Hz, 수신 간격 p99, 빠진 메시지 수를 찍는다.
  [3] 와치독  : 실제 watchdog_node 를 띄우거나, 이 스크립트의 extra 모드로
               같은 QoS 의 구독자 하나를 더 붙인다.

실험 순서 (각 60초, --duration 으로 조절):
  A. [1]+[2] 만 띄우고 결과 기록
  B. [1]+[2]+[3] 띄우고 결과 기록
  C. (선택) --reliable 로 A, B 반복  ← "느린 구독자가 발행자를 막는" 경우 재현

읽는 법:
  - 소비자 Hz 와 빠진 메시지가 A, B 에서 같으면 와치독은 토픽 Hz 를 떨어뜨리지 않은 것.
  - 발행자 publish() p99 가 B 에서 크게 늘면 발행자가 막힌 것(RELIABLE 에서만 기대됨).
  - 다른 장비(Orin→IPC)라면 소비자와 와치독을 IPC 에, 발행자를 Orin 에 둔다.

예:
  python3 hz_ab_test.py pub   --topic /ab/points --hz 10 --bytes 2000000
  python3 hz_ab_test.py sub   --topic /ab/points --hz 10
  python3 hz_ab_test.py extra --topic /ab/points            # 와치독 대역
  python3 hz_ab_test.py pub --reliable ... / sub --reliable ...   # RELIABLE 실험
"""
import argparse
import statistics
import sys
import time


def pct(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * p))]


def qos_of(args):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    return QoSProfile(
        depth=args.depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE if args.reliable else ReliabilityPolicy.BEST_EFFORT)


def make_cloud(nbytes, seq):
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.frame_id = "ab"
    msg.height = 1
    msg.point_step = 16
    msg.width = max(1, nbytes // msg.point_step)
    msg.row_step = msg.width * msg.point_step
    msg.is_dense = True
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "i"))]
    msg.data = bytes(msg.row_step)
    # 첫 4바이트에 순번을 넣어 소비자가 빠진 메시지를 센다
    msg.data = seq.to_bytes(4, "little") + msg.data[4:]
    return msg


def run_pub(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node("ab_pub")
    pub = node.create_publisher(PointCloud2, args.topic, qos_of(args))
    period = 1.0 / args.hz
    lat = []
    seq = 0
    t_end = time.monotonic() + args.duration
    next_t = time.monotonic()
    first_pub = last_pub = None
    print(f"[pub] {args.topic} {args.hz} Hz {args.bytes} bytes "
          f"{'RELIABLE' if args.reliable else 'BEST_EFFORT'} depth={args.depth} {args.duration}s")
    while time.monotonic() < t_end:
        msg = make_cloud(args.bytes, seq)
        msg.header.stamp = node.get_clock().now().to_msg()
        t0 = time.perf_counter()
        pub.publish(msg)
        lat.append((time.perf_counter() - t0) * 1e3)
        last_pub = time.monotonic()
        first_pub = first_pub if first_pub is not None else last_pub
        seq += 1
        next_t += period
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        rclpy.spin_once(node, timeout_sec=0)
    actual_hz = (seq - 1) / max((last_pub or 0) - (first_pub or 0), 1e-9)   # 소비자와 같은 방식(첫~마지막 간격)
    print(f"[pub] sent={seq} 실제 {actual_hz:.2f} Hz (목표 {args.hz}; 소비자 Hz 는 이 값과 비교) "
          f"publish() ms: p50={pct(lat, .5):.3f} "
          f"p99={pct(lat, .99):.3f} max={max(lat):.3f}  "
          f"(> {period * 1e3 / 2:.0f} ms 면 발행 주기를 잡아먹는 수준)")
    node.destroy_node()
    rclpy.shutdown()


def run_sub(args, label):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node(f"ab_{label}")
    arrivals = []
    seqs = []

    def cb(msg):
        arrivals.append(time.monotonic())
        seqs.append(int.from_bytes(bytes(msg.data[:4]), "little"))

    node.create_subscription(PointCloud2, args.topic, cb, qos_of(args))
    print(f"[{label}] {args.topic} {'RELIABLE' if args.reliable else 'BEST_EFFORT'} "
          f"depth={args.depth} {args.duration}s")
    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    if len(arrivals) < 2:
        print(f"[{label}] 수신 {len(arrivals)}개. 발행자가 떠 있는지, QoS 가 맞는지 확인")
        return
    span = arrivals[-1] - arrivals[0]
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    lost = (seqs[-1] - seqs[0] + 1) - len(seqs)
    hz = (len(arrivals) - 1) / span
    print(f"[{label}] rx={len(arrivals)} hz={hz:.2f} (기대 {args.hz}) lost={lost} "
          f"간격 ms: p50={pct(gaps, .5) * 1e3:.1f} p99={pct(gaps, .99) * 1e3:.1f} "
          f"max={max(gaps) * 1e3:.1f} jitter(sd)={statistics.pstdev(gaps) * 1e3:.1f}")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("role", choices=["pub", "sub", "extra"])
    ap.add_argument("--topic", default="/ab/points")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--bytes", type=int, default=2_000_000, help="메시지 크기 (라이다 1프레임 ≈ 1~3 MB)")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--reliable", action="store_true", help="RELIABLE QoS (기본 BEST_EFFORT)")
    ap.add_argument("--depth", type=int, default=5)
    args = ap.parse_args()
    try:
        import rclpy  # noqa: F401
    except ImportError:
        sys.exit("ROS 2 환경이 필요합니다 (source /opt/ros/humble/setup.bash)")
    if args.role == "pub":
        run_pub(args)
    else:
        run_sub(args, "sub" if args.role == "sub" else "extra")


if __name__ == "__main__":
    main()
