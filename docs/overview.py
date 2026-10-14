# README 맨 위 그림(docs/overview.png)을 만드는 스크립트.  python3 docs/overview.py
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.font_manager as fm

names = {f.name for f in fm.fontManager.ttflist}
FONT = next(f for f in ["NanumBarunGothic", "NanumGothic", "Noto Sans CJK KR", "Malgun Gothic", "DejaVu Sans"] if f in names)
plt.rcParams.update({"font.family": FONT, "axes.unicode_minus": False})

fig = plt.figure(figsize=(13, 8.6), dpi=150)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 130); ax.set_ylim(0, 86); ax.axis("off")

C_SENSOR, C_WD, C_SYS, C_OUT, C_NOTE = "#dbeafe", "#fef3c7", "#dcfce7", "#ede9fe", "#f3f4f6"
E_SENSOR, E_WD, E_SYS, E_OUT = "#2563eb", "#d97706", "#16a34a", "#7c3aed"

def box(x, y, w, h, fc, ec, title=None, body=None, tfs=13, bfs=10.5, ls="-", lw=1.6, tcolor="black", body_dy=0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.4", fc=fc, ec=ec, lw=lw, ls=ls))
    if title and body:
        ax.text(x + w/2, y + h - 1.6, title, ha="center", va="top", fontsize=tfs, fontweight="bold", color=tcolor)
        ax.text(x + w/2, y + (h - 4.5)/2 + body_dy, body, ha="center", va="center", fontsize=bfs, linespacing=1.5)
    elif title:
        ax.text(x + w/2, y + h/2, title, ha="center", va="center", fontsize=tfs, fontweight="bold", color=tcolor, linespacing=1.4)
    elif body:
        ax.text(x + w/2, y + h/2, body, ha="center", va="center", fontsize=bfs, linespacing=1.5)

def arrow(p0, p1, color="#374151", lw=2.2, label=None, lx=0, ly=0, fs=10.5, ls="-", conn="arc3,rad=0"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=18, lw=lw, color=color, ls=ls,
                                 shrinkA=0, shrinkB=0, connectionstyle=conn))
    if label:
        ax.text((p0[0]+p1[0])/2 + lx, (p0[1]+p1[1])/2 + ly, label, fontsize=fs, ha="center", va="center",
                color=color, fontweight="bold", bbox=dict(fc="white", ec="none", pad=1.5))

ax.text(65, 83.5, "sensor_watchdog 한눈에 보기 — 센서가 멈추면 알아채고, 안전할 때만 껐다 켠다", ha="center", va="center",
        fontsize=17, fontweight="bold")

# ---------- 1. 센서 ----------
box(3, 58, 48, 20, C_SENSOR, E_SENSOR)
ax.text(27, 76.2, "① 센서 드라이버  (각각 systemd 가 켜 둔 프로그램)", ha="center", va="top", fontsize=13, fontweight="bold", color=E_SENSOR)
box(5.5, 60, 14, 12, "white", E_SENSOR, title="라이다 4대", body="앞·뒤 M1P\n좌·우 Bpearl\n초당 10번\nlidar.service", tfs=12, bfs=9.5)
box(21, 60, 13, 12, "white", E_SENSOR, title="GPS", body="/fix\n초당 20번\n\nrtk-gps.service", tfs=12, bfs=9.5)
box(35.5, 60, 13, 12, "white", E_SENSOR, title="카메라", body="Jetson Orin\n(SSH로 관리)\n\ncam.service", tfs=12, bfs=9.5, ls="--")

# 기어 토픽
box(3, 44, 20, 9.5, "white", "#6b7280", title="차량 기어 상태", body="/vehicle/status/gear_status\n(테스트 중이라 지금은 안 봄)", tfs=11.5, bfs=9, ls="--")

# ---------- 2. 와치독 ----------
box(56, 18, 46, 60, C_WD, E_WD)
ax.text(79, 76.2, "② 와치독  (sensor_watchdog 노드 1개, IPC에서 실행)", ha="center", va="top", fontsize=13, fontweight="bold", color=E_WD)

steps = [
    ("지켜본다  (0.2초마다)",
     "데이터가 오나?  너무 드물게 오나?\n데이터 안의 시계(stamp)가 멈췄나?  크기가 너무 작나?\n* 내용은 안 풀어봄 — 머리글만 읽어서 CPU 거의 안 씀"),
    ("참는다",
     "잠깐 끊긴 건 무시.  끊김 2초 / 느림 5초 이상\n계속돼야 \"고장\"으로 확정"),
    ("안전한지 본다",
     "차가 P단인가?  ← 테스트 중이라 꺼 둠 (gate_topic: \"\")\n한 유닛의 센서가 한꺼번에 죽었나?  → 한 번만 재시작\n방금 재시작한 센서인가?  → 돌아올 때까지 기다림"),
    ("복구 사다리  (한 칸씩 올라감)",
     "1) 노드 lifecycle 재시작  →  2) 프로세스 껐다 켬  →  3) 전원\n몇 번 해도 안 되면 포기하고 \"EXHAUSTED\" 로 보고"),
]
y = 69.5
for i, (t, b) in enumerate(steps):
    h = [10.5, 8, 10.5, 9.5][i]
    box(59, y - h, 40, h, "white", E_WD, title=t, body=b, tfs=12, bfs=9.5, body_dy=0.3)
    if i < 3:
        arrow((79, y - h), (79, y - h - 1.6), color=E_WD, lw=2)
    y = y - h - 1.6

# ---------- 3. systemd ----------
box(3, 3, 99, 9.5, C_SYS, E_SYS)
ax.text(52.5, 10.8, "③ systemd  —  리눅스에 원래 들어 있는 \"프로그램 관리인\"", ha="center", va="top", fontsize=13, fontweight="bold", color=E_SYS)
ax.text(52.5, 5.6, "센서 드라이버를 켜고 · 죽으면 다시 켜고 · 와치독이 부탁하면 껐다 켠다.   와치독 자신도 여기서 관리된다.",
        ha="center", va="center", fontsize=10.5)

# ---------- 4. 출력 ----------
box(106, 48, 22, 30, C_OUT, E_OUT)
ax.text(117, 76.2, "④ 밖으로 알림", ha="center", va="top", fontsize=13, fontweight="bold", color=E_OUT)
box(108, 62, 18, 11, "white", E_OUT, title="/diagnostics", body="센서마다\n정상 / 고장 / 복구중\nrqt 에서 바로 보임", tfs=11.5, bfs=9)
box(108, 50, 18, 10.5, "white", E_OUT, title="CSV 로그", body="메시지마다 한 줄 +\n언제 왜 재시작했는지", tfs=11.5, bfs=9)

# ---------- 화살표 ----------
arrow((51, 68), (56, 68), color=E_SENSOR, lw=2.6, label="토픽 (데이터가 계속 흘러옴)", lx=13, ly=3.5, fs=10)
arrow((23, 48.8), (59, 42.5), color="#6b7280", lw=1.8, label="P단 확인 (꺼 둠)", lx=-6, ly=1.5, fs=9.5, conn="arc3,rad=0.15")
arrow((99, 40), (108, 55.5), color=E_OUT, lw=2.2, conn="arc3,rad=-0.25")
arrow((99, 66), (108, 67.5), color=E_OUT, lw=2.2, label="상태 보고", ly=2, fs=9.5)
# 와치독 -> systemd : 재시작 요청
arrow((79, 26.2), (79, 12.5), color=E_SYS, lw=2.6, label="\"lidar 껐다 켜줘\"  (systemctl --user restart lidar)", lx=0, ly=0, fs=10)
# systemd -> 센서 : 재시작
arrow((27, 12.5), (27, 58), color=E_SYS, lw=2.6, label="센서 프로그램을 껐다 켬", lx=0, ly=-14, fs=10, ls="--")
# 와치독 <-> systemd 하트비트
arrow((96, 18), (96, 12.5), color=E_SYS, lw=1.6, ls=":")
ax.text(103.5, 14.5, "와치독: \"나 살아있어\" (0.2초마다)\n5초간 소식 없으면 systemd 가\n와치독을 다시 띄움", fontsize=8.5, ha="left", va="center", color=E_SYS)

# ---------- 예시 ----------
box(106, 20, 22, 25, C_NOTE, "#9ca3af")
ax.text(117, 43.2, "예: 앞 라이다가 멈추면", ha="center", va="top", fontsize=12, fontweight="bold")
ax.text(108, 38.5,
        "0초   데이터 끊김\n"
        "2초   \"고장\" 확정\n"
        "        (P단 확인은 꺼 둠)\n"
        "        다른 라이다 3대는 멀쩡\n"
        "        → 앞 라이다 문제로 판단\n"
        "3초   systemd 에 재시작 요청\n"
        "        (4대가 한 런치면 4대 같이)\n"
        "~15초 데이터 다시 들어옴\n"
        "        → 정상 복귀, 로그에 기록",
        ha="left", va="top", fontsize=9, linespacing=1.45)

out = __file__.rsplit("/", 1)[0] + "/overview.png"
fig.savefig(out, dpi=150, facecolor="white")
print(out)
