"""라이다 장애물 감지 코리더를 위에서 내려다본(탑뷰) 다이어그램으로 그려서
PNG로 저장하는 개발용 도구입니다. 라이다/모터/카메라 없이 config.py 값만
읽어서 그리므로 PC에서 바로 실행할 수 있습니다.

config.py의 LIDAR_OBSTACLE_LATERAL_MIN/MAX_MM, LIDAR_DETECT_MIN/
MAX_DISTANCE_MM, LIDAR_LANE_WIDTH_MM 값이 바뀌면 그림도 그에 맞게
달라지므로, 값을 조정할 때마다 다시 실행해서 눈으로 확인하는 용도로
쓰면 됩니다.

사용법:
  python visualize_detection_zone.py
  (workspace 루트에 lidar_detection_zone.png로 저장됩니다)
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np

import config as cfg

# 한글 라벨이 네모(□)로 안 깨지도록, 시스템에 있는 한글 지원 폰트를 찾아
# 사용합니다. 못 찾으면 기본 폰트로 진행하되 한글이 깨질 수 있습니다.
for _font_name in ("Noto Sans CJK KR", "NanumGothic", "Malgun Gothic", "AppleGothic"):
    if any(_font_name in f.name for f in fm.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _font_name
        break
matplotlib.rcParams["axes.unicode_minus"] = False

OUTPUT_PATH = "lidar_detection_zone.png"


def main():
    lane_half = cfg.LIDAR_LANE_WIDTH_MM * 0.5  # 우리 차선(2차선) 중앙에서 경계까지
    obstacle_min = cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM
    obstacle_max = cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM
    dist_min = cfg.LIDAR_DETECT_MIN_DISTANCE_MM
    dist_max = cfg.LIDAR_DETECT_MAX_DISTANCE_MM

    # 1차선은 우리 차선(2차선) 바로 안쪽(그림에서 왼쪽)에 붙어있다고 가정합니다.
    lane1_outer = -lane_half           # 1/2차선 중앙선 (우리 차선 안쪽 경계)
    lane1_inner = lane1_outer - cfg.LIDAR_LANE_WIDTH_MM  # 1차선 바깥쪽(트랙 안쪽 끝)

    y_max = dist_max * 1.15
    x_margin = 120.0
    x_min_plot = lane1_inner - x_margin
    x_max_plot = lane_half + x_margin
    car_w, car_l = 90.0, 160.0

    fig, ax = plt.subplots(figsize=(7, 9))

    # 1차선(옆 차선, 무시해야 하는 영역) 배경
    ax.add_patch(plt.Rectangle(
        (lane1_inner, 0), lane1_outer - lane1_inner, y_max,
        color="#333333", alpha=0.35, zorder=0, label="1차선 (옆 차선, 무시)"
    ))
    # 2차선(우리 차선) 배경
    ax.add_patch(plt.Rectangle(
        (-lane_half, 0), 2 * lane_half, y_max,
        color="#1f77b4", alpha=0.15, zorder=0, label="2차선 (우리 차선)"
    ))
    # 실제 장애물 감지 구간 (트랙 좌표 40~64cm -> lateral min~max)
    ax.add_patch(plt.Rectangle(
        (obstacle_min, dist_min),
        obstacle_max - obstacle_min,
        dist_max - dist_min,
        color="#d62728", alpha=0.55, zorder=2,
    ))

    # 차선 경계선들
    for x, style, label in (
        (lane1_inner, "-", "트랙 안쪽 끝\n(0cm)"),
        (lane1_outer, "--", "1·2차선 중앙선\n(34cm)"),
        (lane_half, "-", "2차선 바깥쪽 끝\n(68cm)"),
    ):
        ax.axvline(x, color="black", linewidth=1.2, linestyle=style, zorder=1)
        ax.text(x, -car_l * 0.95, label, ha="center", va="top", fontsize=7)

    # 차량 아이콘 (원점, 정면 = +Y)
    ax.add_patch(plt.Rectangle(
        (-car_w / 2, -car_l * 0.7), car_w, car_l,
        color="#2ca02c", zorder=5,
    ))
    ax.annotate(
        "", xy=(0, car_l * 0.6), xytext=(0, 0),
        arrowprops=dict(arrowstyle="-|>", color="black", linewidth=2),
        zorder=6,
    )
    ax.text(0, -car_l * 0.85, "차량 (라이다 원점)", ha="center", va="top", fontsize=9)

    # 감지거리 최소/최대 원호 표시
    theta = np.linspace(-np.pi / 2, np.pi / 2, 200)
    for r, label in ((dist_min, f"{dist_min:.0f}mm"), (dist_max, f"{dist_max:.0f}mm")):
        ax.plot(r * np.sin(theta), r * np.cos(theta), color="gray", linewidth=0.8, linestyle=":", zorder=1)
        ax.text(0, r, label, fontsize=7, color="gray", ha="center", va="bottom")

    # 감지구간 네 모서리로 향하는 각도 표시 (거리에 따라 각도가 달라짐을 시각화)
    corners = [
        (obstacle_min, dist_min),
        (obstacle_min, dist_max),
        (obstacle_max, dist_min),
        (obstacle_max, dist_max),
    ]
    for x, y in corners:
        angle_deg = np.degrees(np.arctan2(x, y))
        ax.plot([0, x], [0, y], color="#d62728", linewidth=0.8, linestyle="-", alpha=0.7, zorder=3)
        ax.annotate(
            f"{angle_deg:+.1f}°\n({x:.0f}, {y:.0f})",
            xy=(x, y), fontsize=7, color="#8b0000",
            ha="left" if x >= 0 else "right", va="bottom",
        )

    ax.set_xlim(x_min_plot, x_max_plot)
    ax.set_ylim(-car_l * 1.45, y_max * 1.05)
    ax.set_aspect("equal")
    ax.set_xlabel("lateral (mm, 차량 중앙 기준 좌우)")
    ax.set_ylabel("forward (mm, 차량 전방)")
    ax.set_title(
        "라이다 장애물 감지 코리더 (탑뷰)\n"
        f"감지구간: 트랙 {cfg.LIDAR_OBSTACLE_TRACK_MIN_CM:.0f}~"
        f"{cfg.LIDAR_OBSTACLE_TRACK_MAX_CM:.0f}cm "
        f"(lateral {obstacle_min:.0f}~{obstacle_max:.0f}mm), "
        f"거리 {dist_min:.0f}~{dist_max:.0f}mm"
    )
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=8, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.3)

    fig.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=150)
    print(f"저장 완료: {OUTPUT_PATH}")
    print(
        f"참고: 가까운 모서리({dist_min:.0f}mm)의 각도는 "
        f"{np.degrees(np.arctan2(obstacle_min, dist_min)):+.1f}~"
        f"{np.degrees(np.arctan2(obstacle_max, dist_min)):+.1f}도, "
        f"먼 모서리({dist_max:.0f}mm)의 각도는 "
        f"{np.degrees(np.arctan2(obstacle_min, dist_max)):+.1f}~"
        f"{np.degrees(np.arctan2(obstacle_max, dist_max)):+.1f}도로 서로 다릅니다 — "
        "이게 바로 각도창 대신 좌우 폭 코리더 방식을 쓴 이유입니다."
    )


if __name__ == "__main__":
    main()
