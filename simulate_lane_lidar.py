"""차선 마스크·라이다 감지 거리를 하드웨어 없이 검증하는 시뮬레이션.

사용법:
  python simulate_lane_lidar.py
"""

import numpy as np

import config as cfg
from lane_controller import LaneController


def corridor_hit(distance_mm, angle_deg=0.0, offset_lanes=0.0):
    """obstacle_detector._hit_in_lane_corridor와 동일한 판정(단일 점)."""
    angle_diff_rad = np.radians(angle_deg)
    lateral = distance_mm * np.sin(angle_diff_rad)
    forward = distance_mm * np.cos(angle_diff_rad)
    if forward <= 0:
        return False, None
    if not (
        cfg.LIDAR_DETECT_MIN_DISTANCE_MM
        <= distance_mm
        <= cfg.LIDAR_DETECT_MAX_DISTANCE_MM
    ):
        return False, None
    lane_shift = (
        offset_lanes * cfg.AVOID_LANE_DIRECTION * cfg.LIDAR_LANE_WIDTH_MM
    )
    slack = distance_mm * cfg.LIDAR_LATERAL_DISTANCE_SLACK_RATIO
    lat_min = cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM - lane_shift - slack
    lat_max = cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM - lane_shift + slack
    in_lane = lat_min <= lateral <= lat_max
    return in_lane, {
        "distance": distance_mm,
        "lateral": lateral,
        "forward": forward,
        "in_lane": in_lane,
    }


def make_synthetic_track_frame(width=640, height=480):
    """회색 바닥 + 흰 차선(오른쪽 실선, 왼쪽 점선 조각) 합성 영상."""
    frame = np.full((height, width, 3), (145, 145, 145), dtype=np.uint8)
    noise = np.random.default_rng(0).integers(-12, 12, (height, width), dtype=np.int16)
    gray = np.clip(155 + noise, 0, 255).astype(np.uint8)
    frame[:, :, 0] = gray
    frame[:, :, 1] = gray
    frame[:, :, 2] = gray

    for y in range(height):
        x_right = int(420 + (y - height) * 0.08)
        x_left = int(180 + (y - height) * 0.05)
        for dx in range(-2, 3):
            xr = x_right + dx
            xl = x_left + dx
            if 0 <= xr < width:
                frame[y, xr] = (250, 250, 250)
            if 0 <= xl < width and (y // 28) % 2 == 0:
                frame[y, xl] = (248, 248, 248)
    return frame


def make_lane1_with_parking(width=640, height=480):
    """1차선: 주차장 선(왼쪽) + 외곽 실선 + 중앙 점선."""
    frame = np.full((height, width, 3), 118, dtype=np.uint8)
    for y in range(height):
        x_parking = int(35 + (y - height) * 0.01)
        x_left = int(130 + (y - height) * 0.04)
        x_center = int(360 + (y - height) * 0.07)
        for dx in range(-2, 3):
            xp = x_parking + dx
            xl = x_left + dx
            xc = x_center + dx
            if 0 <= xp < width:
                frame[y, xp] = (240, 240, 240)
            if 0 <= xl < width:
                frame[y, xl] = (245, 245, 245)
            if 0 <= xc < width and (y // 24) % 2 == 0:
                frame[y, xc] = (250, 250, 250)
    return frame


def simulate_mask():
    import cv2

    frame = make_synthetic_track_frame()
    roi, mask, _ = LaneController._make_mask(frame)
    white_ratio = np.count_nonzero(mask) / mask.size
    segments = LaneController._row_segments(
        mask, mask.shape[0] * cfg.CONTROL_Y_RATIOS[0]
    )
    widths = [s["width"] for s in segments]
    print("=== 차선 마스크 시뮬레이션 ===")
    print(f"흰 픽셀 비율: {white_ratio * 100:.1f}%")
    print(f"제어 높이 세그먼트 수: {len(segments)}")
    if widths:
        print(f"세그먼트 폭(px): min={min(widths)}, max={max(widths)}")
    cv2.imwrite("simulate_mask_result.png", mask)
    print("저장: simulate_mask_result.png")
    return white_ratio < 0.25


def simulate_lane1_left_mask():
    """1차선: 중앙 점선 왼쪽에서 가장 오른쪽 실선을 왼쪽 차선으로 잡는지 검증."""
    frame = make_lane1_with_parking()
    lc = LaneController()
    lc.set_lane_offset(cfg.AVOID_LANE_OFFSET_LANES)

    roi, mask, _ = LaneController._make_mask(frame)
    height, width = mask.shape
    rows = np.linspace(
        height * cfg.TRACK_TOP_RATIO,
        height * cfg.TRACK_BOTTOM_RATIO,
        cfg.TRACK_ROW_COUNT,
    )
    center_rows = lc._track_lane_one_center_rows(mask, rows, height, width)
    left_track = lc._track_lane_one_left_boundary(center_rows, width)
    picked_left_x = int(np.median([item["left"] for item in left_track])) if left_track else -1

    for _ in range(cfg.TRACK_START_CONFIRM_FRAMES):
        result = lc.update(frame)

    print("\n=== 1차선 왼쪽 실선 시뮬레이션 ===")
    print(f"중앙 점선 추적 행: {len(center_rows)}")
    print(f"왼쪽 실선 추적 행: {len(left_track)}")
    print(f"선택된 왼쪽 x(중앙값): {picked_left_x} (주차장~35, 외곽실선~130)")
    print(
        f"left_primary={lc._avoid_left_primary_active()} "
        f"status={result.get('status')} "
        f"conf={result.get('confidence', 0):.2f}"
    )

    ok = (
        len(left_track) >= cfg.SINGLE_LANE_MIN_POINTS
        and picked_left_x > 90
        and result.get("status") not in ("LOST", "SINGLE_RIGHT")
    )
    return ok


def simulate_lidar_distances():
    print("\n=== 라이다 거리 시뮬레이션 ===")
    print(
        f"설정 감지거리: {cfg.LIDAR_DETECT_MIN_DISTANCE_MM:.0f}~"
        f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm"
    )
    print(
        f"감지 구간(트랙): {cfg.LIDAR_OBSTACLE_TRACK_MIN_CM:.0f}~"
        f"{cfg.LIDAR_OBSTACLE_TRACK_MAX_CM:.0f}cm"
    )

    ok_150 = False
    ok_50 = False
    for distance_mm in (1500, 500):
        hit, debug = corridor_hit(distance_mm)
        label = "150cm" if distance_mm == 1500 else "50cm"
        print(
            f"  {label}: hit={hit} "
            f"lat={debug['lateral']:.0f} in_lane={debug['in_lane']}"
            if debug
            else f"  {label}: hit={hit} (no debug)"
        )
        if distance_mm == 1500:
            ok_150 = hit
        else:
            ok_50 = hit

    angle_err_deg = 2.0
    hit, debug = corridor_hit(1500.0, angle_deg=2.0)
    print(
        f"  150cm + {angle_err_deg}° 오차: hit={hit} "
        f"lat={debug['lateral']:.0f} in_lane={debug['in_lane']}"
        if debug
        else f"  150cm + {angle_err_deg}° 오차: hit={hit}"
    )
    return ok_150 and ok_50


def main():
    mask_ok = simulate_mask()
    lane1_ok = simulate_lane1_left_mask()
    lidar_ok = simulate_lidar_distances()
    print("\n=== 결과 ===")
    print(f"마스크(바닥 덩어리 억제): {'PASS' if mask_ok else 'CHECK'}")
    print(f"1차선 왼쪽 실선 추적: {'PASS' if lane1_ok else 'FAIL'}")
    print(f"라이다 150cm/50cm 감지: {'PASS' if lidar_ok else 'CHECK'}")
    if cfg.LIDAR_FRONT_ANGLE == 0.0:
        print(
            "\n주의: LIDAR_FRONT_ANGLE=0.0(미보정)이면 실차에서 멀수록 "
            "lateral이 어긋날 수 있습니다. calibrate_lidar_angle.py로 "
            "보정하세요."
        )


if __name__ == "__main__":
    main()
