# sensor_watchdog

ROS 2 Humble 용 센서 감시·자동 복구 노드입니다. 라이다, GPS, 카메라 같은 센서 토픽이 끊기면 알아채고, 정해진 조건에서만 센서 프로세스를 껐다 켜며, 센서 상태를 `/diagnostics` 로 계속 알립니다.

- 센서 프로세스 관리는 리눅스 기본 관리자인 **systemd** 에 맡깁니다. 별도 셸 스크립트 없음.
- 장비마다 자기 센서만 감시합니다. 원격 장비(Jetson Orin 등)에 SSH 로 들어가지 않습니다.
- 처음엔 **감지 전용 모드**(`dry_run: true`)로 돌려서 오탐이 얼마나 되는지 확인한 뒤에 실제 재시작을 켭니다.

> 상태: 판정 로직은 단위 테스트를 통과했습니다. 실제 ROS 2 Humble 빌드와 실차 검증은 아직 하지 않았습니다. 자세한 건 [검증 범위](#검증-범위와-한계) 참고.

---

## 목차

1. [무엇을 해결하나](#무엇을-해결하나)
2. [30초 요약: 어떻게 동작하나](#30초-요약-어떻게-동작하나)
3. [구성도](#구성도)
4. [파일 구조](#파일-구조)
5. [설치](#설치)
6. [매일 쓰는 명령](#매일-쓰는-명령)
7. [로그 읽는 법](#로그-읽는-법)
8. [동작 원리 자세히](#동작-원리-자세히)
9. [설정 레퍼런스](#설정-레퍼런스)
10. [실제 재시작을 켜기 전 절차](#실제-재시작을-켜기-전-절차)
11. [안전 게이트 설정](#안전-게이트-설정)
12. [안전 MCU 하트비트](#안전-mcu-하트비트)
13. [자주 묻는 질문](#자주-묻는-질문)
14. [테스트](#테스트)
15. [검증 범위와 한계](#검증-범위와-한계)

---

## 무엇을 해결하나

실차에서 센서 드라이버는 가끔 멈춥니다. 프로세스는 살아 있는데 데이터만 안 나오는 경우가 특히 골치입니다. 사람이 터미널을 보고 있다가 Ctrl+C 로 끄고 다시 켜는 건 주행 중엔 불가능합니다.

이 패키지는 그 일을 대신합니다. 단, 아무 때나 재시작하면 더 위험하기 때문에 다음 원칙을 지킵니다.

| 원칙 | 구현 |
|---|---|
| 확실히 끊긴 것만 건드린다 | 메시지가 **3초** 동안 하나도 안 오거나, 발행자/노드가 사라진 경우만 재시작. 주기가 떨어지거나 타임스탬프가 이상한 건 기록만 |
| 센서 탓이 아니면 건드리지 않는다 | 여러 센서가 동시에 죽으면 네트워크나 호스트 문제로 보고 재시작하지 않음 |
| 재시작은 가벼운 것부터 | lifecycle 재활성화 → 서비스 재시작 → 전원 리셋 순서. 단계당 2회 |
| 무한 반복하지 않는다 | 사다리 끝까지 실패하면 `EXHAUSTED` 로 표시하고 사람을 기다림 |
| 다른 노드가 알 수 있어야 한다 | 상태를 표준 `/diagnostics` 토픽으로 1초마다 발행 |
| 감시하는 놈도 감시받는다 | systemd 가 5초마다 생존 확인. 작은 MCU 에 UDP 하트비트 |
| 켜기 전에 데이터부터 본다 | 메시지마다 원시 CSV 기록. 감지 전용 모드로 운행 후 기준 충족 시 활성화 |

---

## 30초 요약: 어떻게 동작하나

```
센서 토픽 ──▶ 와치독 (0.2초마다 검사)
                │
                ├─ 정상?  → /diagnostics 에 OK
                │
                ├─ 1초 끊김 → /diagnostics 에 ERROR (플래너가 알 수 있음)
                │
                └─ 3초 끊김 → "systemctl --user restart lidar" 실행
                               │
                               └─ systemd 가 런치 안의 모든 노드를 끄고 다시 켬
                                  10초 유예 후 다시 검사
```

와치독은 센서를 직접 죽이지 않습니다. systemd 한테 "저거 껐다 켜줘" 하고 부탁만 합니다. 실제로 프로세스를 끄고 켜는 건 systemd 가 하고, 자식 프로세스까지 남김없이 정리합니다.

---

## 구성도

```
┌─────────────────────────────────────────────────────────────┐
│ IPC                                                         │
│                                                             │
│   systemd ─┬─ lidar.service     (ros2 launch ... 라이다)    │
│            ├─ rtk-gps.service   (ros2 launch ... GPS)       │
│            └─ sensor-watchdog.service                       │
│                   │  watchdog_ipc, ipc.yaml                  │
│                   ├─ 라이다·GPS 토픽 구독                     │
│                   ├─ /diagnostics 발행 ──────▶ 플래너        │
│                   ├─ 게이트 토픽 구독 (예: /ad_can/gear)      │
│                   └─ UDP 하트비트 ───────────▶ 안전 MCU      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Orin (192.168.x.x)                                          │
│                                                             │
│   systemd ─┬─ cam.service       (ros2 launch ... 카메라)    │
│            └─ sensor-watchdog.service                       │
│                   watchdog_orin, orin.yaml  (같은 바이너리)  │
└─────────────────────────────────────────────────────────────┘
```

IPC 와 Orin 은 서로 들어가지 않습니다. 각자 자기 센서만 봅니다. 두 와치독의 `/diagnostics` 는 같은 DDS 도메인이라 한 곳에서 다 보입니다.

---

## 파일 구조

```
sensor_watchdog/
├── include/sensor_watchdog/
│   ├── health.hpp          판정 로직 + 복구 사다리 + 공통 원인 판정 (ROS 의존 없음)
│   └── cdr_header.hpp      직렬화된 메시지에서 header.stamp 와 첫 정수 필드만 읽기
├── src/watchdog.cpp        ROS 노드. 구독, /diagnostics 발행, systemctl 호출, 기록
├── test/test_health.cpp    ROS 없이 돌아가는 단위 테스트
├── config/
│   ├── ipc.yaml            IPC 용 (lidar, gps)
│   └── orin.yaml           Orin 용 (camera)
├── systemd/
│   ├── lidar.service       센서 런치를 systemd 서비스로
│   ├── rtk-gps.service
│   ├── cam.service
│   └── sensor-watchdog.service
├── scripts/
│   ├── env.bash.example    서비스가 source 할 ROS 환경 파일 틀
│   └── power_cycle.sh.example   전원 리셋 스크립트 틀 (수단이 있을 때만)
├── CMakeLists.txt
└── package.xml
```

---

## 설치

### 0. 준비물

- Ubuntu 22.04 + ROS 2 Humble
- 센서 드라이버가 이미 `ros2 launch` 로 돌아가는 상태
- 감시할 토픽 이름과 타입. 모르면 센서 켠 상태에서:

```bash
ros2 topic list -t                        # 토픽 이름과 타입
ros2 topic info /토픽이름 --verbose       # QoS (Reliability, Durability)
ros2 topic hz /토픽이름                   # 주기
```

### 1. 빌드 (IPC, Orin 둘 다)

```bash
sudo apt install -y build-essential python3-colcon-common-extensions \
  ros-humble-rclcpp ros-humble-diagnostic-msgs ros-humble-lifecycle-msgs \
  ros-humble-sensor-msgs coreutils

mkdir -p ~/watchdog_ws/src
cd ~/watchdog_ws/src
git clone https://github.com/utsi09/sensor_watchdog.git
cd ~/watchdog_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sensor_watchdog
colcon test --packages-select sensor_watchdog && colcon test-result --verbose
```

### 2. 설정 파일 복사

```bash
mkdir -p ~/.config/sensor_watchdog ~/.config/systemd/user
cd ~/watchdog_ws/src/sensor_watchdog

cp scripts/env.bash.example ~/.config/sensor_watchdog/env.bash
cp config/ipc.yaml          ~/.config/sensor_watchdog/watchdog.yaml   # Orin 이면 orin.yaml
cp systemd/*.service        ~/.config/systemd/user/
```

### 3. 빈칸 채우기 (세 파일)

**`~/.config/sensor_watchdog/env.bash`** — 센서 켤 때 필요한 환경. `.bashrc` 전체를 넣지 말고 필요한 줄만.

```bash
source /opt/ros/humble/setup.bash
source /home/USER/sensor_ws/install/setup.bash    # 드라이버 워크스페이스
# export ROS_DOMAIN_ID=0
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**`~/.config/sensor_watchdog/watchdog.yaml`** — `CHANGE_ME` 를 실제 값으로.

```yaml
lidar:
  topic: /velodyne_points                # 0번에서 확인한 이름
  type: sensor_msgs/msg/PointCloud2
  unit: lidar.service
  qos:
    reliability: best_effort             # ros2 topic info --verbose 와 맞춤
    durability: volatile
```

**`~/.config/systemd/user/lidar.service`** — `ExecStart` 에 평소 치던 런치 명령.

```ini
ExecStart=/bin/bash -c 'source %h/.config/sensor_watchdog/env.bash && exec ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py'
```

평소 alias 로 켰다면 `alias lidar` 를 치면 실제 명령이 나옵니다. 그걸 그대로 넣으면 됩니다.

### 4. systemd 에 알리기, 기존 센서 끄기

```bash
systemctl --user daemon-reload
```

터미널에서 띄워둔 센서가 있으면 Ctrl+C 로 전부 끕니다. 안 끄면 두 개가 동시에 돌아갑니다.

### 5. 켜기

```bash
systemctl --user start lidar rtk-gps          # 센서
systemctl --user start sensor-watchdog        # 와치독 (감지 전용 모드)
journalctl --user -u sensor-watchdog -f       # 로그 보기
```

`[lidar] HEALTHY` 가 뜨면 성공입니다.

### 6. (선택) 부팅 시 자동 시작

```bash
loginctl enable-linger $USER                  # 로그인 안 해도 user 서비스 유지. 한 번만
systemctl --user enable lidar rtk-gps sensor-watchdog
```

자동 시작이 싫으면 `enable` 을 하지 않고 매번 `start` 만 쓰면 됩니다.

### Orin 은?

1~6 을 똑같이 하되 `config/orin.yaml` 과 `cam.service` 를 쓰고, `sensor-watchdog.service` 안의 `NODE=watchdog_ipc` 를 `NODE=watchdog_orin` 으로 바꿉니다.

```bash
sed -i 's/NODE=watchdog_ipc/NODE=watchdog_orin/' ~/.config/systemd/user/sensor-watchdog.service
```

---

## 매일 쓰는 명령

| 하고 싶은 것 | 명령 |
|---|---|
| 센서 켜기 | `systemctl --user start lidar` |
| 센서 끄기 | `systemctl --user stop sensor-watchdog` 먼저, 그다음 `systemctl --user stop lidar` |
| 센서 껐다 켜기 | `systemctl --user restart lidar` |
| 살아 있나 | `systemctl --user status lidar` |
| 센서 로그 | `journalctl --user -u lidar -f` |
| 와치독 로그 | `journalctl --user -u sensor-watchdog -f` |
| 상태 토픽 | `ros2 topic echo /diagnostics` |
| 원시 기록 | `tail -f ~/.local/state/sensor_watchdog/lidar.csv` |
| 사건 기록 | `tail -f ~/.local/state/sensor_watchdog/events.csv` |

**끄는 순서 주의.** 와치독이 켜진 채로 센서만 끄면, 와치독이 "발행자가 사라졌다"고 보고 2초 뒤 다시 켭니다.

---

## 로그 읽는 법

`journalctl --user -u sensor-watchdog -f` 에 나오는 줄의 뜻:

| 로그 | 뜻 | 할 일 |
|---|---|---|
| `[lidar] GRACE` | 시작 직후 10초 유예. 아직 판정 안 함 | 기다림 |
| `[lidar] HEALTHY` | 정상 | 없음 |
| `[lidar] HOLDING: RX_TIMEOUT` | 1초 이상 끊김. 2초 더 지켜보는 중 | 보통 혼자 돌아옴 |
| `[lidar] COOLDOWN: RX_TIMEOUT` | 고장인데 직전 재시작 후 30초 안 지남 | 기다림 |
| `[lidar] MONITOR_ONLY: LOW_RX_HZ` | 고장이지만 재시작 대상 아님 (주기 저하 등) | 로그만 확인 |
| `[lidar] BLOCKED_COMMON_CAUSE` | 여러 센서가 동시에 죽음. 센서 탓 아님 | 네트워크·호스트 확인 |
| `[lidar] BLOCKED_GATE` | 차량 쪽이 재시작 허가 안 함 | 정차 후 자동 진행 |
| `[lidar] DRY_RUN_WOULD_service` | 감지 전용 모드. "원래면 지금 재시작했다" | 오탐인지 확인 |
| `[lidar] ACTION restart lidar.service (step 1, attempt 1, total 1)` | 실제 재시작 중 | 10초 뒤 결과 확인 |
| `[lidar] action done exit=0; re-checking after grace` | 명령 끝남. 성공 여부는 다음 판정에서 | |
| `[lidar] EXHAUSTED: RX_TIMEOUT — human needed` | 사다리 끝까지 실패 | 직접 확인 |
| `NO_MATCHED_PUBLISHER_CHECK_QOS_TYPE` | 발행자는 있는데 나한테 안 옴 | yaml 의 type, qos 를 `ros2 topic info --verbose` 와 맞춤 |

고장 코드 전체:

| 코드 | 뜻 | 재시작 대상 (기본) |
|---|---|---|
| `NODE_NOT_VISIBLE` | 설정한 노드 이름이 그래프에 없음 | O |
| `NO_PUBLISHER` | 토픽 발행자가 없음 | O |
| `NO_MATCHED_PUBLISHER_CHECK_QOS_TYPE` | 발행자는 있으나 QoS/타입 불일치로 매칭 안 됨 | X (설정 문제) |
| `RX_TIMEOUT` | `timeout_sec` 동안 메시지 없음 | O |
| `TOO_SMALL` | 메시지가 `min_bytes` 보다 작음 | X |
| `STAMP_INVALID_OR_BACKWARD` | 타임스탬프가 비정상이거나 뒤로 감 | X |
| `STAMP_AGE_OR_CLOCK` | 메시지가 너무 오래됨 (`check_stamp_age` 켰을 때만) | X |
| `STAMP_NOT_ADVANCING` | 메시지는 오는데 타임스탬프가 안 바뀜 | X |
| `LOW_RX_HZ` | 수신 주기가 `min_hz` 미만 | X |

---

## 동작 원리 자세히

### 1. 듣기: 메시지를 풀지 않고 헤더만 읽는다

와치독은 `GenericSubscription` 으로 토픽을 받습니다. 메시지를 C++ 객체로 풀지(역직렬화) 않고, 직렬화된 바이트에서 `header.stamp` 위치만 읽습니다. 포인트클라우드가 메가바이트 단위여도 와치독 CPU 부하는 거의 없습니다.

sensor_msgs 의 PointCloud2, Image, CompressedImage, NavSatFix, Imu 모두 첫 필드가 `std_msgs/Header` 라서 같은 코드로 처리됩니다. 설정의 `type` 문자열만 바꾸면 됩니다.

메시지가 올 때마다 세 가지를 적습니다.

- 도착 시각 (`steady_clock`. 시스템 시각이 바뀌어도 안 튐)
- 메시지 안의 타임스탬프
- 바이트 크기

### 2. 판정: 0.2초마다 10가지 검사

위에서부터 순서대로 보고, 처음 걸리는 것 하나만 보고합니다.

```
1. 노드가 그래프에 있나?          (node_name 설정 시)
2. 발행자가 있나?
3. 내 구독과 매칭됐나?            (QoS/타입)
4. timeout_sec 안에 메시지 왔나?   ← "끊김"의 핵심
5. 크기가 min_bytes 이상인가?
6. 타임스탬프가 정상인가?
7. 타임스탬프가 너무 오래되지 않았나? (check_stamp_age 켰을 때)
8. 타임스탬프가 전진하나?
9. 수신 주기가 min_hz 이상인가?
→ 전부 통과하면 HEALTHY
```

### 3. "끊겼다"의 정의

```
메시지 끊김
  ↓ 1.0초 (timeout_sec)        RX_TIMEOUT 판정. /diagnostics 에 ERROR
  ↓ 2.0초 (fault_hold_sec)     고장이 계속되면
  = 3.0초                      재시작
```

중간에 메시지가 하나라도 오면 처음부터 다시 셉니다. 연속 3초여야 합니다.

발행자 없음·노드 없음은 그래프를 보고 즉시 알 수 있어서 `fault_hold_sec` 만 적용돼 약 2초입니다. 단, 드라이버가 강제로 죽거나 멈춘 경우 DDS 가 그걸 알아채는 데 10~20초가 걸릴 수 있어서, 실제로는 거의 항상 `RX_TIMEOUT` 쪽이 먼저 잡습니다.

### 4. 재시작 전 세 가지 관문

`ACT` 판정이 나도 바로 재시작하지 않습니다.

1. **고장 종류가 `act_on_faults` 에 있는가.** 기본은 `RX_TIMEOUT`, `NO_PUBLISHER`, `NODE_NOT_VISIBLE` 세 가지. 나머지는 `MONITOR_ONLY`.
2. **공통 원인이 아닌가.** `common_cause_window_sec`(3초) 안에 `common_cause_min`(2개) 이상 센서가 함께 죽었으면 `BLOCKED_COMMON_CAUSE`. 네트워크나 호스트 문제에 센서를 재시작해 봐야 소용없기 때문입니다.
3. **안전 게이트가 허용했는가.** `gate_topic` 을 설정하면 그 값(예: 기어 P)이 맞을 때만. 단, 고장이 `gate_bypass_after_sec` 이상 계속되면 게이트를 무시합니다. 기본값은 `fault_hold_sec` 와 같아서 "확실히 끊긴 센서는 기어와 무관하게 재시작"됩니다.
4. **`dry_run` 이 꺼져 있는가.** 켜져 있으면 `DRY_RUN_WOULD_*` 만 기록.

### 5. 복구 사다리

```
lifecycle  →  service  →  power  →  EXHAUSTED
(ms 단위)     (초 단위)    (수십 초)   (사람 호출)
```

- **lifecycle**: 드라이버가 ROS 2 lifecycle 노드일 때만. 프로세스는 그대로 두고 `deactivate → activate`. 가장 빠름. `lifecycle_node` 설정 시에만 사다리에 포함.
- **service**: `systemctl --user restart <unit>`. 프로세스를 끄고 다시 켬. `unit` 설정 시에만.
- **power**: `power_cmd` 로 지정한 스크립트 실행. PoE 포트 off/on, USB 허브 리셋 등 센서 장치 전원만 껐다 켬. IPC 나 Orin 을 재부팅하는 게 아님. `power_cmd` 설정 시에만.

같은 칸을 `attempts_per_step`(2회) 시도해도 안 되면 다음 칸. 끝까지 실패하면 `EXHAUSTED`. `healthy_reset_sec`(10분) 동안 정상이면 사다리를 처음으로 되돌립니다.

각 행동 뒤에는 `startup_grace_sec`(10초) 유예 후 다시 판정하고, `cooldown_sec`(30초) 동안은 다음 행동을 하지 않습니다.

### 6. 상태 발행: `/diagnostics`

1초마다 `diagnostic_msgs/DiagnosticArray` 를 발행합니다. 센서마다 하나, 그리고 요약 하나.

```
name: "watchdog_ipc: lidar"
level: 0=OK, 1=WARN(유예 중), 2=ERROR(고장)
message: "HOLDING: RX_TIMEOUT"
values: fault, state, rx_hz, age_sec, rx_count, step, attempts_at_step, total_actions, exhausted, dry_run
```

플래너는 `level` 만 보면 됩니다. `ERROR` 면 그 센서를 지금 쓰면 안 됩니다.

### 7. 기록

`raw_log_dir` 아래에 두 종류.

- `<센서>.csv`: 메시지마다 한 줄. `steady, ros_now, stamp_ns, bytes`. 임계값을 실측으로 정할 때 씁니다.
- `events.csv`: 상태 변화, 행동 시작·종료, 복구까지 걸린 시간, 게이트 우회.

### 8. 와치독 자체 감시

- systemd `WatchdogSec=5`: 와치독이 0.2초마다 `WATCHDOG=1` 을 보냅니다. 5초 끊기면 systemd 가 죽이고 다시 띄웁니다.
- UDP 하트비트: `heartbeat_host` 를 설정하면 0.2초마다 MCU 에 한 줄 보냅니다. MCU 는 끊기면 IPC 에 묻지 않고 스스로 안전 정지합니다. [규약](#안전-mcu-하트비트) 참고.

---

## 설정 레퍼런스

`config/ipc.yaml` 기준. 모든 값은 ROS 파라미터라 `--ros-args -p dry_run:=false` 처럼 명령줄에서 덮어쓸 수 있습니다.

### 전역

| 키 | 기본값 | 뜻 |
|---|---|---|
| `dry_run` | `true` | true 면 판정·기록만 하고 행동 안 함 |
| `raw_log_dir` | `~/.local/state/sensor_watchdog` | 원시 CSV 와 events.csv 위치. `""` 면 끔 |
| `gate_topic` | `""` | 재시작 허가 토픽. `""` 면 항상 허용 |
| `gate_type` | `std_msgs/msg/Bool` | 게이트 토픽 타입 |
| `gate_has_header` | `false` | 게이트 메시지 첫 필드가 `std_msgs/Header` 인가 |
| `gate_field_bytes` | `1` | 헤더 다음 첫 필드 크기 (1, 2, 4, 8) |
| `gate_ok_value` | `1` | 이 값이면 허용 |
| `gate_reliable` | `true` | 게이트 구독 QoS |
| `gate_timeout_sec` | `1.0` | 이 시간 안에 게이트 메시지가 와야 유효 |
| `gate_bypass_after_sec` | `2.0` | 고장이 이만큼 지속되면 게이트 무시. 음수면 항상 게이트 따름 |
| `common_cause_min` | `2` | 이 개수 이상 동시 고장이면 공통 원인 |
| `common_cause_window_sec` | `3.0` | "동시"의 기준 |
| `action_timeout_sec` | `30.0` | 한 행동에 허용하는 최대 시간 |
| `systemctl_cmd` | `["systemctl", "--user"]` | 서비스 재시작 명령 앞부분. 시스템 유닛이면 `["sudo", "-n", "systemctl"]` |
| `heartbeat_host` | `""` | MCU IPv4 주소. `""` 면 끔 |
| `heartbeat_port` | `5005` | |
| `startup_grace_sec` | `10.0` | 시작·행동 직후 판정 유예 |
| `fault_hold_sec` | `2.0` | 고장 표시 후 이만큼 더 지속돼야 행동 |
| `act_on_faults` | `[RX_TIMEOUT, NO_PUBLISHER, NODE_NOT_VISIBLE]` | 행동하는 고장 종류 |
| `cooldown_sec` | `30.0` | 행동 완료 후 다음 행동까지 최소 간격 |
| `attempts_per_step` | `2` | 같은 칸 시도 횟수 |
| `healthy_reset_sec` | `600.0` | 이만큼 정상이면 사다리 초기화 |
| `ladder` | `[lifecycle, service, power]` | 복구 순서 |
| `sensors` | | 감시할 센서 id 목록 |

### 센서별 (`<id>.` 아래)

| 키 | 기본값 | 뜻 |
|---|---|---|
| `topic` | 필수 | `/` 로 시작 |
| `type` | 필수 | 예: `sensor_msgs/msg/PointCloud2`. 첫 필드가 Header 인 타입 |
| `node_name` | `""` | 설정하면 노드 존재 검사. `/` 로 시작 |
| `lifecycle_node` | `""` | lifecycle 노드 이름. 설정 시 사다리에 lifecycle 칸 포함 |
| `unit` | `""` | systemd 유닛 이름. 설정 시 service 칸 포함 |
| `power_cmd` | 없음 | 전원 리셋 명령 배열. 설정 시 power 칸 포함. 빈 배열 `[]` 은 쓰지 말고 줄을 지울 것 |
| `timeout_sec` | `1.0` | 이 시간 메시지 없으면 RX_TIMEOUT |
| `stamp_timeout_sec` | `1.0` | 타임스탬프가 이 시간 안 바뀌면 STAMP_NOT_ADVANCING |
| `min_hz` | `0.0` | 수신 주기 하한. 0 이면 검사 안 함 |
| `hz_window_sec` | `5.0` | 주기 계산 구간 |
| `min_bytes` | `1` | 메시지 크기 하한 |
| `check_stamp_progress` | `true` | 타임스탬프 전진 검사 |
| `check_stamp_age` | `false` | 타임스탬프 나이 검사. 장비 간 시각 동기(PTP) 후 켤 것 |
| `max_age_sec` | `0.5` | 나이 상한 |
| `qos.reliability` | `best_effort` | `best_effort` 또는 `reliable`. 발행자와 호환돼야 함 |
| `qos.durability` | `volatile` | `volatile` 또는 `transient_local` |
| `qos.depth` | `5` | |

---

## 실제 재시작을 켜기 전 절차

이 숫자들(1초, 3초, 5 Hz)은 전부 예시입니다. 실제 센서에 맞는 값은 측정해서 정해야 합니다.

### 1. 예상값을 먼저 적는다

| 센서 | 예상 수신 간격 (평균 / 최대) | 예상 크기 (바이트) | 하루 끊김 횟수 예상 | 근거 |
|---|---|---|---|---|
| lidar | | | | |
| gps | | | | |
| camera | | | | |

적어두고 나중에 원시 로그와 비교합니다. 맞으면 설정값에 근거가 생기고, 틀리면 뭘 몰랐는지 알게 됩니다.

### 2. 기본 동작을 실측한다 (각 5분, 스톱워치)

1. `systemctl --user restart lidar` 후 `ros2 topic hz` 가 다시 뜨기까지 몇 초인가 → `startup_grace_sec` 근거
2. 라이다 케이블을 뽑았다 꽂으면 드라이버가 혼자 살아나는가. 산다면 재시작이 필요 없는 고장이다 → `fault_hold_sec` 근거
3. `systemctl --user stop lidar` 후 `pgrep -af <드라이버패키지>` 에 남는 프로세스가 있는가 → `KillMode` 검증

### 3. 감지 전용 운행

`dry_run: true` 로 며칠 운행합니다. `events.csv` 의 `DRY_RUN_WOULD_*` 줄을 하나씩 보고 "진짜 고장이었나, 오탐이었나"를 적습니다.

### 4. 활성화 기준

아래를 모두 만족하면 `dry_run: false` 로 바꿉니다.

- 진짜 고장이 10회 이상 관측됐다
- 오탐이 진짜 고장의 절반 이하다
- 예상값 표와 원시 로그의 수신 간격 분포가 일치했다. 불일치했다면 원인을 기록했다
- 플래너가 `/diagnostics` ERROR 를 받으면 감속·정지하는 것을 정지 상태에서 확인했다

```bash
# watchdog.yaml 에서 dry_run: false 로 바꾼 뒤
systemctl --user restart sensor-watchdog
```

---

## 안전 게이트 설정

차량 쪽 코드를 고치지 않고 기존 토픽(예: 기어 상태)을 그대로 씁니다.

```bash
ros2 topic type /ad_can/gear              # 예: ad_can_msgs/msg/Gear
ros2 interface show ad_can_msgs/msg/Gear  # 첫 필드 타입과 P 에 해당하는 상수
ros2 topic echo /ad_can/gear --once       # 실제 값
```

```yaml
gate_topic: /ad_can/gear
gate_type: ad_can_msgs/msg/Gear
gate_has_header: false        # 첫 줄이 std_msgs/Header 면 true
gate_field_bytes: 1           # 기어 필드가 uint8 이면 1, int32 면 4
gate_ok_value: 0              # P 에 해당하는 값
gate_reliable: true           # ros2 topic info --verbose 의 Reliability 와 맞춤
```

제약: 기어 값이 (헤더를 제외하고) 메시지의 **첫 번째 필드**여야 합니다. 다른 위치면 `src/watchdog.cpp` 의 게이트 파싱을 그 타입에 맞게 바꿔야 합니다.

게이트 판정 결과는 `/diagnostics` 요약 항목의 `gate` 값에서 볼 수 있습니다 (`allowed value=0`, `blocked value=3`, `no message (blocked)`).

---

## 안전 MCU 하트비트

`heartbeat_host` 를 설정하면 0.2초마다 UDP 로 한 줄을 보냅니다.

```
WD <틱번호> <정상수>/<센서수> <최악레벨>\n
예) WD 1234 2/2 0
```

최악레벨: `0` 모두 정상, `1` 유예 중인 센서 있음, `2` 고장 센서 있음.

MCU 쪽 규칙 (펌웨어는 이 패키지 범위 밖):

- 1초 이상 패킷이 없으면 → IPC 또는 와치독이 죽은 것. 안전 정지
- 레벨 2가 N초 이상 지속되면 → 안전 정지

MCU 는 IPC 에 묻지 않고 스스로 판단합니다. 지키는 쪽이 지켜지는 쪽 위에 살면 안 되기 때문입니다.

---

## 자주 묻는 질문

**systemd 가 뭔가요?**
리눅스에 기본으로 들어 있는 프로그램 관리자입니다. 부팅할 때 네트워크, SSH 등을 띄우는 게 systemd 입니다. 이 패키지는 센서 런치를 systemd 서비스로 등록해서 켜고 끄고, 죽으면 되살리고, 재시작할 때 자식 프로세스까지 정리하는 일을 systemd 에 맡깁니다. 와치독이 하는 "재시작"은 `systemctl --user restart lidar` 한 줄입니다.

**systemd 와 와치독이 둘 다 재시작하면 충돌하지 않나요?**
드라이버가 아예 죽으면(세그폴트) systemd 가 `Restart=on-failure` 로 3초 뒤 되살립니다. 와치독도 3초쯤 `restart` 를 보내지만 같은 유닛이라 한 번만 뜹니다. 드라이버가 살아 있는데 데이터만 안 나오는 경우(행)는 systemd 가 모르고 와치독만 잡습니다. 역할이 나뉩니다.

**부팅 시 자동으로 안 띄우고 싶어요.**
`enable` 을 하지 않고 `start` 만 쓰면 됩니다. 이미 했다면 `systemctl --user disable lidar`. 죽었을 때 되살아나는 것도 끄려면 유닛 파일에서 `Restart=no`.

**와치독을 끄면 센서도 꺼지나요?**
아니요. 와치독은 감시만 합니다. 센서는 systemd 가 들고 있어서 그대로 돕니다.

**ROS 2 의 Deadline / Liveliness QoS 를 쓰면 되지 않나요?**
그게 DDS 가 제공하는 "와치독용 QoS" 가 맞습니다. 안 쓴 이유는 발행자(센서 드라이버)도 같은 설정을 해야 매칭되기 때문입니다. 드라이버 기본값은 "무한" 이라, 구독자가 1초 Deadline 을 요구하면 아예 연결이 안 됩니다. 드라이버를 안 고치는 조건에서는 타이머로 직접 재는 수밖에 없습니다. 드라이버를 직접 만든다면 Deadline 으로 바꾸는 게 깔끔합니다.

**전원 리셋이 IPC 재부팅인가요?**
아닙니다. 센서 장치 하나의 전원만 끊었다 잇는 것입니다. PoE 스위치 포트, USB 허브 포트, 릴레이 등. 그런 수단이 없으면 `power_cmd` 를 설정하지 않으면 되고, 그러면 그 단계는 자동으로 빠집니다.

**주기가 떨어진 센서는 왜 재시작하지 않나요?**
조금이라도 오는 센서를 재시작하면 10초 이상 완전히 끊깁니다. "반쯤 오는 것"과 "안 오는 것" 중 어느 쪽이 나은지는 상황마다 달라서, 기본은 기록만 하고 판단은 사람에게 맡깁니다. 재시작하고 싶으면 `act_on_faults` 에 `LOW_RX_HZ` 를 추가하면 됩니다.

**카메라가 `sensor_msgs/msg/Image` 인데요?**
`type` 만 바꾸면 됩니다. 코드 수정 없음. 첫 필드가 `std_msgs/Header` 인 타입이면 뭐든 됩니다.

---

## 테스트

판정 로직은 ROS 없이 테스트합니다. ROS 가 없는 PC 에서도 돌릴 수 있습니다.

```bash
g++ -std=c++17 -Wall -Wextra -I include test/test_health.cpp -o test_health
./test_health
# ALL PASSED
```

colcon 으로는 `colcon test --packages-select sensor_watchdog`.

테스트 항목: CDR 헤더 파싱(LE/BE), 정상 흐름, 수신 타임아웃, 그래프 고장 우선순위, 타임스탬프 정지·역행·무효, 크기 미달, 주기 저하, 타임스탬프 나이, 사다리 전체 흐름(유예→대기→행동→다음 칸→소진→초기화), 사다리 없음, 공통 원인, 게이트 필드 파싱.

---

## 검증 범위와 한계

**검증됨**
- `test/test_health.cpp` 전부 통과 (g++ 16, Windows)
- `src/watchdog.cpp` 문법 검사 통과 (모의 rclcpp 헤더 사용)

**검증 안 됨**
- 실제 ROS 2 Humble 환경에서 colcon 빌드. 모의 헤더와 실제 API 가 다른 부분이 있으면 첫 빌드에서 컴파일 오류가 날 수 있습니다. 특히 `create_generic_subscription` 과 lifecycle 서비스 콜백 부분
- systemd 유닛의 실제 동작
- 실제 센서, 실차

**설계상 검출하지 않는 것**
- 메시지는 정상이나 내용이 쓸모없는 경우 (역광으로 날아간 영상, 눈 덮인 라이다, 멀티패스 GPS). 원시 로그에서 사례가 확인되면 그때 추가합니다
- 가상 환경(시뮬레이션) 시각. `use_sim_time` 은 지원하지 않습니다

**지원 타입**
- 첫 필드가 `std_msgs/Header` 인 모든 메시지. sensor_msgs 의 PointCloud2, Image, CompressedImage, NavSatFix, Imu 등

---

## 라이선스

Apache-2.0
