#!/usr/bin/env python3
"""scripts/bag_diag_check.py 판정 로직을 ROS 없이 가짜 백으로 검증한다.

실행:  python3 test/test_bag_diag_check.py
ROS 모듈(rosbag2_py, rclpy, diagnostic_msgs)을 가짜로 바꿔 끼운 뒤 main() 을 호출하고
출력 문자열을 확인한다. 각 시나리오는 "이렇게 되어야 한다"를 먼저 적고 확인한다.
"""
import contextlib
import io
import os
import struct
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

T0 = 1_700_000_000.0
DIAG = "velodyne_driver: velodyne_packets topic status"


def stamp_bytes(t, little=True):
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    if little:
        return b"\x00\x01\x00\x00" + struct.pack("<iI", sec, nsec) + b"\x00" * 8
    return b"\x00\x00\x00\x00" + struct.pack(">iI", sec, nsec) + b"\x00" * 8


class Status:
    def __init__(self, name, level, message):
        self.name, self.level, self.message = name, level, message


class Array:
    def __init__(self, status):
        self.status = status


class FakeBag:
    """msgs: (topic, payload_bytes, t_sec). diag_payloads: payload_bytes -> Array"""

    def __init__(self):
        self.msgs = []
        self.diag_payloads = {}
        self.types = {}

    def sensor(self, topic, t, stamp=None, little=True):
        self.types[topic] = "sensor_msgs/msg/PointCloud2"
        self.msgs.append((topic, stamp_bytes(T0 + (t if stamp is None else stamp), little),
                          int((T0 + t) * 1e9)))

    def sensor_stream(self, topic, hz, t_from, t_to, skip=lambda t: False, stuck_from=None):
        n = int(round((t_to - t_from) * hz))
        for i in range(n):
            t = t_from + i / hz
            if skip(t):
                continue
            stamp = t if stuck_from is None or t < stuck_from else stuck_from
            self.sensor(topic, t, stamp)

    def diag(self, t, statuses):
        self.types["/diagnostics"] = "diagnostic_msgs/msg/DiagnosticArray"
        key = f"diag{len(self.diag_payloads)}".encode()
        self.diag_payloads[key] = Array([Status(n, lv, m) for n, lv, m in statuses])
        self.msgs.append(("/diagnostics", key, int((T0 + t) * 1e9)))

    def diag_stream(self, t_from, t_to, level_at, period=1.0):
        t = t_from
        while t < t_to:
            lv = level_at(t)
            if lv is not None:
                self.diag(t, [(DIAG, lv, "ok" if lv == 0 else "no events in window"),
                              ("other: unrelated", 0, "ok")])
            t += period

    def install(self):
        self.msgs.sort(key=lambda m: m[2])
        bag = self

        class Meta:
            def __init__(self, name, type_):
                self.name, self.type = name, type_

        class SequentialReader:
            def __init__(self):
                self.i = 0

            def open(self, so, co):
                if so.storage_id != "sqlite3":
                    raise RuntimeError("only sqlite3")

            def get_all_topics_and_types(self):
                return [Meta(k, v) for k, v in bag.types.items()]

            def has_next(self):
                return self.i < len(bag.msgs)

            def read_next(self):
                m = bag.msgs[self.i]
                self.i += 1
                return m

        class StorageOptions:
            def __init__(self, uri, storage_id):
                self.uri, self.storage_id = uri, storage_id

        class ConverterOptions:
            def __init__(self, a, b):
                pass

        m = types.ModuleType
        rosbag2_py = m("rosbag2_py")
        rosbag2_py.SequentialReader = SequentialReader
        rosbag2_py.StorageOptions = StorageOptions
        rosbag2_py.ConverterOptions = ConverterOptions
        rclpy_ser = m("rclpy.serialization")
        rclpy_ser.deserialize_message = lambda data, T: bag.diag_payloads[data]
        dmsg = m("diagnostic_msgs.msg")
        dmsg.DiagnosticArray = Array
        sys.modules.update({"rosbag2_py": rosbag2_py, "rclpy": m("rclpy"),
                            "rclpy.serialization": rclpy_ser,
                            "diagnostic_msgs": m("diagnostic_msgs"),
                            "diagnostic_msgs.msg": dmsg})


def run(bag, *argv):
    bag.install()
    import bag_diag_check
    out = io.StringIO()
    sys.argv = ["bag_diag_check.py", "fake", *argv]
    with contextlib.redirect_stdout(out):
        try:
            bag_diag_check.main()
        except SystemExit as e:
            out.write(f"\n[SystemExit {e.code}]")
    return out.getvalue()


def section(text, n):
    """n번 단락만 잘라 낸다."""
    start = text.index(f"=== {n}.")
    nxt = text.find(f"=== {n + 1}.", start)
    return text[start:nxt if nxt >= 0 else None]


class BagDiagCheckTest(unittest.TestCase):

    def test_gap_caught_by_error(self):
        """3초 끊김. diag 가 1.4초 뒤 ERROR → '잡힘', 지연 +1.40s. 오탐 없음."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 15, skip=lambda t: 4.0 < t < 7.0)
        b.diag_stream(0.4, 15, lambda t: 2 if 5.4 <= t < 7.4 else 0)
        out = run(b, "--topic", "/lidar/points", "--timeout", "1.0", "--keyword", "velodyne")
        self.assertIn("끊김 1회", out)
        self.assertIn("4.00s ~     7.00s  (3.00s)", out)
        self.assertIn("잡힘", section(out, 3))
        self.assertNotIn("못 잡음", out)
        self.assertIn("ERROR 지연 +1.40s", out)
        self.assertIn("(없음)", section(out, 4))

    def test_gap_caught_by_stale(self):
        """드라이버가 통째로 멈춰 diag 보고 자체가 끊긴 경우 → '보고끊김' 으로 잡힘."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 20, skip=lambda t: 5.0 < t < 12.0)
        # diag 는 5.4 부터 11.4 까지 아예 안 나옴. 그 전후는 OK.
        b.diag_stream(0.4, 20, lambda t: None if 5.4 <= t < 11.4 else 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne", "--diag-timeout", "3.0")
        self.assertIn("잡힘", section(out, 3))
        self.assertIn("보고끊김", out)
        # 마지막 보고 4.4 + 3.0 = 7.4, 끊김 시작 5.0 → 지연 +2.40
        self.assertIn("지연 +2.40s", out)

    def test_gap_not_caught(self):
        """diag 가 계속 OK 만 내면 '못 잡음'. (드라이버가 주기 검사를 안 하는 경우)"""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 15, skip=lambda t: 4.0 < t < 7.0)
        b.diag_stream(0.4, 15, lambda t: 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne")
        self.assertIn("못 잡음", section(out, 3))
        self.assertNotIn("잡힘\n", section(out, 3).replace("못 잡음", ""))

    def test_no_related_diag_name(self):
        """센서 관련 diag 이름이 없으면 'diag 모드 불가' 안내."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 15, skip=lambda t: 4.0 < t < 7.0)
        b.diag_stream(0.4, 15, lambda t: 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "ouster")
        self.assertIn("diag 모드 불가", out)
        self.assertIn("못 잡음", out)

    def test_no_diagnostics_topic(self):
        """백에 /diagnostics 가 없으면 '확인 불가' 결론을 먼저 찍고도 끊김 분석은 한다."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 15, skip=lambda t: 4.0 < t < 7.0)
        out = run(b, "--topic", "/lidar/points")
        self.assertIn("확인 불가", out)
        self.assertIn("끊김 1회", out)

    def test_false_positive_reported(self):
        """토픽은 정상인데 diag 만 ERROR 인 시각은 4단락에 나와야 한다."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 20)
        b.diag_stream(0.4, 20, lambda t: 2 if abs(t - 13.4) < 1e-6 else 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne")
        self.assertIn("끊김 0회", out)
        self.assertIn("13.40s " + DIAG, section(out, 4))

    def test_error_right_after_gap_is_not_false_positive(self):
        """끊김 복구 직후 diag 가 잠깐 더 ERROR 인 건 정상(창 회복 지연). 오탐으로 안 센다."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 20, skip=lambda t: 4.0 < t < 7.0)
        b.diag_stream(0.4, 20, lambda t: 2 if 5.4 <= t < 9.4 else 0)   # 7.0 복구 후 9.4 까지 ERROR
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne")
        self.assertIn("(없음)", section(out, 4))

    def test_end_of_bag_gap(self):
        """백 끝까지 메시지가 안 오면 그것도 끊김."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 5)
        b.diag_stream(0.4, 12, lambda t: 2 if t >= 6.4 else 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne")
        self.assertIn("끊김 1회", out)
        self.assertIn("4.90s ~    11.40s", out)
        self.assertIn("잡힘", section(out, 3))

    def test_stuck_stamp_counted(self):
        """프로세스는 살아서 같은 stamp 를 반복 발행 → stamp 비전진 경고."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 10, stuck_from=6.0)
        b.diag_stream(0.4, 10, lambda t: 0)
        out = run(b, "--topic", "/lidar/points", "--keyword", "velodyne")
        self.assertIn("끊김 0회", out)
        self.assertIn("stamp 비전진/역행 메시지 39개", out)   # 6.1 ~ 9.9 → 39개

    def test_big_endian_header(self):
        """빅엔디안 CDR 도 stamp 를 읽는다 (비전진 검사가 돌아가야 함)."""
        b = FakeBag()
        for i in range(50):
            b.sensor("/imu", i * 0.1, stamp=5.0, little=False)   # 전부 같은 stamp
        b.diag_stream(0.4, 5, lambda t: 0)
        out = run(b, "--topic", "/imu", "--keyword", "velodyne")
        self.assertIn("stamp 비전진/역행 메시지 49개", out)

    def test_two_topics_with_own_timeouts(self):
        """토픽마다 timeout 이 따로 적용된다. GPS 는 3초 끊김이 timeout 3.0 안이라 정상."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 15, skip=lambda t: 4.0 < t < 7.0)
        b.sensor_stream("/gps/fix", 1, 0, 15, skip=lambda t: 4.0 < t < 6.5)
        b.diag_stream(0.4, 15, lambda t: 2 if 5.4 <= t < 7.4 else 0)
        out = run(b, "--topic", "/lidar/points", "--timeout", "1.0",
                  "--topic", "/gps/fix", "--timeout", "3.0", "--keyword", "velodyne")
        sec2 = section(out, 2)
        self.assertIn("/lidar/points: n=121 평균 8.1 Hz, 끊김 1회", sec2)
        self.assertIn("/gps/fix: n=13 평균 0.9 Hz, 끊김 0회", sec2)

    def test_missing_topic_warned(self):
        """백에 없는 토픽을 주면 경고하고 죽지 않는다."""
        b = FakeBag()
        b.sensor_stream("/lidar/points", 10, 0, 5)
        b.diag_stream(0.4, 5, lambda t: 0)
        out = run(b, "--topic", "/nope")
        self.assertIn("[경고] 백에 없는 토픽: ['/nope']", out)
        self.assertIn("/nope: 메시지 없음", out)
        self.assertNotIn("SystemExit", out)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    unittest.main(verbosity=2)
