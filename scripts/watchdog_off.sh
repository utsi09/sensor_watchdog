#!/usr/bin/env bash
# 와치독 끄기.  사용: scripts/watchdog_off.sh
#
#   systemd 유닛(sensor-watchdog.service)이 돌고 있으면 그걸 끄고,
#   scripts/watchdog_on.sh 가 직접 띄운 프로세스가 있으면 그것도 끈다 (SIGINT → 5초 뒤 SIGKILL).
#   센서 드라이버(lidar.service 등)는 건드리지 않는다.
set -uo pipefail

STATE_DIR="$HOME/.local/state/sensor_watchdog"
PID_FILE="$STATE_DIR/watchdog.pid"
UNIT=sensor-watchdog
done=0

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
