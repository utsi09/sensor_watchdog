#!/usr/bin/env bash
# 와치독 끄기.  사용: scripts/watchdog_off.sh [--all]
#
#   systemd 유닛(sensor-watchdog.service)이 돌고 있으면 그걸 끄고,
#   scripts/watchdog_on.sh 가 직접 띄운 프로세스가 있으면 그것도 끈다 (SIGINT → 5초 뒤 SIGKILL).
#   기본은 센서 드라이버(lidar.service 등)를 건드리지 않는다.
#   --all : 와치독이 재시작해서 띄운 센서 유닛까지 끈다. 목록은 와치독이
#           ~/.local/state/sensor_watchdog/started_units 에 적어 둔 것(한 줄에 "유닛<TAB>systemctl 명령")이고,
#           사람이 직접 띄운 유닛은 목록에 없으므로 건드리지 않는다. 와치독을 먼저 끄고 나서 유닛을 끈다.
set -uo pipefail

STATE_DIR="$HOME/.local/state/sensor_watchdog"
PID_FILE="$STATE_DIR/watchdog.pid"
STARTED_FILE="$STATE_DIR/started_units"
UNIT=sensor-watchdog
ALL=0
for a in "$@"; do
    case "$a" in
        --all) ALL=1 ;;
        -h|--help) sed -n 2,9p "$0"; exit 0 ;;
        *) echo "모르는 인자: $a" >&2; exit 2 ;;
    esac
done
done=0

# 1) 와치독 먼저. 센서를 먼저 끄면 와치독이 "발행자가 사라졌다"고 보고 다시 켠다.
if systemctl --user is-active --quiet "$UNIT" 2>/dev/null; then
    systemctl --user stop "$UNIT"
    echo "systemd 유닛 $UNIT 껐음"
    done=1
fi

if [ -f "$PID_FILE" ]; then
    pid="$(cat "$PID_FILE")"
    if kill -0 "$pid" 2>/dev/null; then
        kill -INT "$pid"
        for _ in $(seq 1 50); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid"
            echo "pid $pid 가 5초 안에 안 꺼져 강제 종료"
        else
            echo "pid $pid 껐음"
        fi
        done=1
    fi
    rm -f "$PID_FILE"
fi

if [ "$done" = 0 ]; then
    echo "켜져 있는 와치독이 없습니다."
fi

# 2) --all : 와치독이 띄운 유닛
if [ "$ALL" = 1 ]; then
    if [ ! -s "$STARTED_FILE" ]; then
        echo "와치독이 띄운 센서 유닛이 없습니다 ($STARTED_FILE 없음)."
        exit 0
    fi
    fail=0
    while IFS=$'\t' read -r unit prefix; do
        [ -n "$unit" ] || continue
        read -ra cmd <<< "${prefix:-systemctl --user}"
        if "${cmd[@]}" stop "$unit"; then
            echo "와치독이 띄운 $unit 껐음 (${cmd[*]} stop $unit)"
        else
            echo "$unit 끄기 실패: ${cmd[*]} stop $unit" >&2
            fail=1
        fi
    done < "$STARTED_FILE"
    [ "$fail" = 0 ] && rm -f "$STARTED_FILE"
    exit "$fail"
fi
