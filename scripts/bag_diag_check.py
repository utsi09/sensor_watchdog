#!/usr/bin/env python3
"""로스백에서 "센서 토픽 끊김"과 "/diagnostics 보고"를 시간순으로 대조한다.

질문: 와치독이 무거운 센서 토픽 대신 /diagnostics 만 봤어도 같은 고장을 잡았을까?

사용 (ROS 2 Humble 이 source 된 셸에서):
  python3 bag_diag_check.py <bag_dir> \
      --topic /sensing/lidar/top/pointcloud_raw --timeout 1.0 \
      --topic /sensing/camera/front/image_raw   --timeout 1.0 \
      [--diag-timeout 3.0] [--keyword lidar --keyword camera]

출력:
  1. 백에 있는 /diagnostics 이름 목록과 level 분포 (어떤 드라이버가 diag 를 내는지)
  2. 센서 토픽별 끊김 구간 (수신 간격 > timeout)
  3. 각 끊김마다, 관련 diag 이름이 ERROR 로 바뀌거나 보고가 끊긴 시각과 지연(초)
  4. 토픽은 정상인데 diag 만 ERROR 였던 구간 (오탐 후보)

판정 기준:
  - 끊김마다 "잡힘" 이고 지연이 fault_hold_sec 안이면 diag 모드로 충분하다.
  - /diagnostics 자체가 백에 없거나 센서 관련 이름이 없으면, 드라이버에 diagnostic_updater 를
    붙이기 전까지 diag 모드는 쓸 수 없다 (그 센서는 topic 모드 유지).
"""
import argparse
import os
import struct
import sys
from collections import defaultdict

LEVEL = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def import_ros():
    """인자 파싱 뒤에 임포트해서 --help 는 ROS 없이도 뜨게 한다."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from diagnostic_msgs.msg import DiagnosticArray
    except ImportError as e:
        sys.exit(f"ROS 2 환경이 필요합니다 (source /opt/ros/humble/setup.bash): {e}")
    return SequentialReader, StorageOptions, ConverterOptions, deserialize_message, DiagnosticArray


def cdr_header_stamp(buf):
    """직렬화 바이트에서 header.stamp 만 읽는다 (cdr_header.hpp 와 같은 규칙)."""
    if len(buf) < 12 or buf[0] != 0 or buf[1] > 1:
        return None
    fmt = "<iI" if buf[1] == 1 else ">iI"
    sec, nsec = struct.unpack_from(fmt, buf, 4)
    return sec + nsec * 1e-9


def storage_hint(path):
    """metadata.yaml 의 storage_identifier (mcap/sqlite3). 없으면 None."""
    meta = os.path.join(path, "metadata.yaml")
    try:
        with open(meta, encoding="utf-8") as f:
            for line in f:
                if "storage_identifier:" in line:
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return None


def open_bag(path, SequentialReader, StorageOptions, ConverterOptions):
    # metadata 가 말해 주는 저장 형식을 먼저 연다. 엉뚱한 플러그인부터 시도하면
    # rosbag2 가 "file is not a database" 같은 ERROR 를 찍어서 헷갈린다.
    hint = storage_hint(path)
    order = [hint] + [s for s in ("sqlite3", "mcap") if s != hint] if hint else ["sqlite3", "mcap"]
    for storage in order:
        reader = SequentialReader()
        try:
            reader.open(StorageOptions(uri=path, storage_id=storage),
                        ConverterOptions("cdr", "cdr"))
            return reader
        except Exception:
            pass
    sys.exit(f"백을 열 수 없음: {path}")


def gaps_of(times, timeout, bag_end):
    """수신 간격이 timeout 을 넘는 구간 [(start, end, gap_sec)]"""
    out = []
    for a, b in zip(times, times[1:]):
        if b - a > timeout:
            out.append((a, b, b - a))
    if times and bag_end - times[-1] > timeout:
        out.append((times[-1], bag_end, bag_end - times[-1]))
    return out


def level_int(level):
    return int.from_bytes(level, "little") if isinstance(level, bytes) else int(level)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")   # Windows 콘솔에서 한글 깨짐 방지
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", action="append", default=[],
                    help="감시 대상 센서 토픽 (여러 개 가능)")
    ap.add_argument("--timeout", action="append", type=float, default=[],
                    help="--topic 과 같은 순서. 생략하면 1.0")
    ap.add_argument("--diag-timeout", type=float, default=3.0,
                    help="diag 보고가 이 시간 이상 안 오면 '보고 끊김' 으로 본다")
    ap.add_argument("--keyword", action="append", default=[],
                    help="센서와 관련된 diag 이름 키워드 (생략하면 토픽 이름 조각으로 추정)")
    ap.add_argument("--diag-topic", default="/diagnostics")
    args = ap.parse_args()
    if not args.topic:
        sys.exit("--topic 을 하나 이상 주세요")
    timeouts = {t: (args.timeout[i] if i < len(args.timeout) else 1.0)
                for i, t in enumerate(args.topic)}

    (SequentialReader, StorageOptions, ConverterOptions,
     deserialize_message, DiagnosticArray) = import_ros()
    reader = open_bag(args.bag, SequentialReader, StorageOptions, ConverterOptions)
    types ={m.name: m.type for m in reader.get_all_topics_and_types()}
    missing = [t for t in args.topic if t not in types]
    if missing:
        print(f"[경고] 백에 없는 토픽: {missing}")
    if args.diag_topic not in types:
        print(f"[결론] 백에 {args.diag_topic} 가 없습니다. diag 모드로 잡혔는지 확인 불가.\n"
              f"       다음 기록부터 'ros2 bag record {args.diag_topic}' 를 포함하세요.")

    rx = defaultdict(list)          # topic -> [bag_time]
    stamp_rx = defaultdict(list)    # topic -> [(bag_time, header_stamp)]
    diag = defaultdict(list)        # name -> [(bag_time, level, message)]
    t0 = None
    t_end = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        t0 = t if t0 is None else t0
        t_end = t
        if topic in timeouts:
            rx[topic].append(t)
            st = cdr_header_stamp(data)
            if st is not None:
                stamp_rx[topic].append((t, st))
        elif topic == args.diag_topic:
            arr = deserialize_message(data, DiagnosticArray)
            for s in arr.status:
                diag[s.name].append((t, level_int(s.level), s.message))

    if t0 is None:
        sys.exit("빈 백")

    def rel(x):
        return f"{x - t0:8.2f}s"

    # 1. diag 이름 목록
    print("\n=== 1. /diagnostics 이름 (개수, level 분포) ===")
    if not diag:
        print("  (없음)")
    for name, rows in sorted(diag.items()):
        hist = defaultdict(int)
        for _, lv, _ in rows:
            hist[LEVEL.get(lv, str(lv))] += 1
        print(f"  {name}  n={len(rows)}  {dict(hist)}")

    # 2. 센서 끊김
    print("\n=== 2. 센서 토픽 끊김 (수신 간격 > timeout) ===")
    all_gaps = {}
    for topic in args.topic:
        times = rx.get(topic, [])
        if not times:
            print(f"  {topic}: 메시지 없음")
            continue
        hz = len(times) / max(times[-1] - times[0], 1e-9)
        g = gaps_of(times, timeouts[topic], t_end)
        all_gaps[topic] = g
        print(f"  {topic}: n={len(times)} 평균 {hz:.1f} Hz, 끊김 {len(g)}회")
        for a, b, d in g:
            print(f"     {rel(a)} ~ {rel(b)}  ({d:.2f}s)")
        # stamp 가 멈춘 경우(프로세스는 살아서 같은 메시지를 반복 발행)
        pairs = zip(stamp_rx[topic], stamp_rx[topic][1:])
        stuck = sum(1 for (_, s1), (_, s2) in pairs if s2 <= s1)
        if stuck:
            print(f"     stamp 비전진/역행 메시지 {stuck}개 "
                  f"(diag 가 timestamp 검사도 하는지 확인 필요)")

    def related_names(topic):
        kws = args.keyword or [p for p in topic.strip("/").split("/") if len(p) > 2]
        return [n for n in diag if any(k.lower() in n.lower() for k in kws)]

    # 3. 끊김 <-> diag 대조
    print("\n=== 3. 끊김마다 diag 로 잡혔는지 ===")
    for topic, g in all_gaps.items():
        names = related_names(topic)
        print(f"  {topic}  관련 diag 이름 {len(names)}개: "
              f"{names if names else '(없음 -> diag 모드 불가)'}")
        for a, b, d in g:
            caught = []
            for n in names:
                rows = diag[n]
                # (a) ERROR 전환
                err = next((t for t, lv, _ in rows if t >= a and lv >= 2), None)
                # (b) 보고 자체가 끊김
                stale = None
                prev = None
                for t, _, _ in rows:
                    if (prev is not None and t - prev > args.diag_timeout
                            and prev >= a - args.diag_timeout):
                        stale = prev + args.diag_timeout
                        break
                    prev = t
                if (rows and stale is None
                        and t_end - rows[-1][0] > args.diag_timeout
                        and rows[-1][0] >= a - args.diag_timeout):
                    stale = rows[-1][0] + args.diag_timeout
                cands = [x for x in (err, stale) if x is not None]
                if cands:
                    first = min(cands)
                    how = "ERROR" if first == err else "보고끊김"
                    caught.append(f"{n} {how} 지연 {first - a:+.2f}s")
            verdict = "잡힘" if caught else "못 잡음"
            print(f"     끊김 {rel(a)} ({d:.2f}s): {verdict}")
            for c in caught:
                print(f"        - {c}")

    # 4. 오탐 후보
    print("\n=== 4. 토픽은 정상인데 diag 가 ERROR 였던 구간 (오탐 후보) ===")
    any_fp = False
    for topic, g in all_gaps.items():
        for n in related_names(topic):
            for t, lv, msg in diag[n]:
                near_gap = any(a - args.diag_timeout <= t <= b + args.diag_timeout
                               for a, b, _ in g)
                if lv >= 2 and not near_gap:
                    any_fp = True
                    print(f"  {rel(t)} {n}: {msg}")
                    break
    if not any_fp:
        print("  (없음)")


if __name__ == "__main__":
    main()
