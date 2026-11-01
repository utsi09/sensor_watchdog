#!/usr/bin/env bash
# 와치독 켜기.  사용: scripts/watchdog_on.sh [설정.yaml] [--live]
#
#   systemd 유닛(sensor-watchdog.service)이 설치돼 있으면 그걸 켠다 (실차 기본).
#   없으면 바이너리를 직접 백그라운드로 띄우고 pid 를 ~/.local/state/sensor_watchdog/watchdog.pid 에 적는다.
#   --live : dry_run 을 false 로 덮어써서 실제 재시작까지 한다 (기본은 설정 파일 값 = 보통 감지 전용).
#
# 끄기는 scripts/watchdog_off.sh
set -euo pipefail

STATE_DIR="$HOME/.local/state/sensor_watchdog"
PID_FILE="$STATE_DIR/watchdog.pid"
LOG_FILE="$STATE_DIR/watchdog.log"
UNIT=sensor-watchdog
NODE=${NODE:-watchdog_ipc}

CONFIG=""
LIVE=0
for a in "$@"; do
    case "$a" in
        --live) LIVE=1 ;;
        -h|--help) sed -n 2,8p "$0"; exit 0 ;;
        *) CONFIG="$a" ;;
    esac
done

mkdir -p "$STATE_DIR"

# 1) systemd 유닛이 있으면 그걸 쓴다
if systemctl --user cat "$UNIT" >/dev/null 2>&1; then
    if [ -n "$CONFIG" ] || [ "$LIVE" = 1 ]; then
        echo "systemd 유닛으로 켭니다. 설정/--live 인자는 유닛 환경(CONFIG, 설정 파일의 dry_run)을 따르므로 무시됩니다." >&2
    fi
    systemctl --user start "$UNIT"
    sleep 1
    systemctl --user --no-pager status "$UNIT" | head -5
    exit 0
fi

# 2) 유닛이 없으면 직접 띄운다
if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "이미 켜져 있습니다 (pid $(cat "$PID_FILE")). 끄려면 scripts/watchdog_off.sh" >&2
    exit 0
fi

HERE="$(cd "$(dirname "$0")/.." && pwd)"
[ -n "$CONFIG" ] || CONFIG="$HOME/.config/sensor_watchdog/watchdog.yaml"
[ -f "$CONFIG" ] || CONFIG="$HERE/config/ipc.yaml"
[ -f "$HOME/.config/sensor_watchdog/env.bash" ] && source "$HOME/.config/sensor_watchdog/env.bash"
if ! command -v ros2 >/dev/null 2>&1; then
    source /opt/ros/humble/setup.bash
fi
BIN="$(ros2 pkg prefix sensor_watchdog 2>/dev/null || true)/lib/sensor_watchdog/watchdog_node"
if [ ! -x "$BIN" ]; then
    # 워크스페이스가 source 안 된 경우: 이 스크립트가 있는 워크스페이스의 install 을 찾는다
    WS="$(cd "$HERE/../.." && pwd)"
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    BIN="$(ros2 pkg prefix sensor_watchdog 2>/dev/null || true)/lib/sensor_watchdog/watchdog_node"
fi
[ -x "$BIN" ] || { echo "watchdog_node 를 찾을 수 없습니다. colcon build 후 install/setup.bash 를 source 하세요." >&2; exit 1; }

EXTRA=()
[ "$LIVE" = 1 ] && EXTRA=(-p dry_run:=false)

nohup "$BIN" --ros-args -r "__node:=$NODE" --params-file "$CONFIG" "${EXTRA[@]}" >> "$LOG_FILE" 2>&1 &
echo $! > "$PID_FILE"
sleep 1
if kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "켜짐: pid $(cat "$PID_FILE"), 설정 $CONFIG, dry_run=$([ "$LIVE" = 1 ] && echo false || echo '설정 파일 값'), 로그 $LOG_FILE"
    tail -n 3 "$LOG_FILE"
else
    echo "기동 실패. 로그:" >&2
    tail -n 20 "$LOG_FILE" >&2
    rm -f "$PID_FILE"
    exit 1
fi
