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
    # 회색 바닥에 약한 밝기 변화(오인식 유발)
    noise = np.random.default_rng(0).integers(-12, 12, (height, width), dtype=np.int16)
    gray = np.clip(155 + noise, 0, 255).astype(np.uint8)
    frame[:, :, 0] = gray
    frame[:, :, 1] = gray
    frame[:, :, 2] = gray

    # 오른쪽 실선 + 왼쪽 점선 조각
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


def make_dashed_right_test_mask(width=640, height=480):
    """점선 오른쪽·가짜 실선 오른쪽 시나리오용 직접 마스크."""
    mask = np.zeros((height, width), dtype=np.uint8)
    for y in range(int(height * 0.12), int(height * 0.95), 8):
        mask[y - 2:y + 3, 352:362] = 255
        mask[y - 2:y + 3, 548:558] = 255
        if (y // 20) % 2 == 0:
            mask[y - 2:y + 3, 446:456] = 255
    return mask


def simulate_dashed_right_solid_selection():
    """점선 오른쪽일 때 점선 왼쪽 실선 중 가장 오른쪽을 고르는지 검증."""
    controller = LaneController()
    controller.set_lane_offset(1.0)
    mask = make_dashed_right_test_mask()
    measurements = controller._collect_measurements(mask)
    if not measurements:
        print("=== 점선-오른쪽 실선 선택 ===")
        print("FAIL: 측정값 없음")
        return False

    control_y = mask.shape[0] * cfg.CONTROL_Y_RATIOS[0]
    near = min(measurements, key=lambda item: abs(item["y"] - control_y))
    dashed_map = controller._track_dashed_positions(
        mask,
        np.linspace(
            mask.shape[0] * cfg.TRACK_TOP_RATIO,
            mask.shape[0] * cfg.TRACK_BOTTOM_RATIO,
            cfg.TRACK_ROW_COUNT,
        ),
    )
    dashed_x = dashed_map.get(near["y"])
    ok = (
        near["right"] is not None
        and dashed_x is not None
        and near["right"] < dashed_x
        and 340 <= near["right"] <= 370
        and near["left"] is not None
        and abs(near["left"] - near["right"]) < 1.0
        and near["source"] == "BOTH"
    )
    left_text = (
        f"{near['left']:.0f}" if near["left"] is not None else "None"
    )
    print("=== 점선-오른쪽 실선 선택 ===")
    print(
        f"right={near['right']:.0f} dashed={dashed_x:.0f} "
        f"left={left_text} source={near['source']}"
    )
    print(f"결과: {'PASS' if ok else 'FAIL'}")
    return ok


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

    lateral_center = (
        cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM
        + cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM
    ) * 0.5

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

    # 중앙 lateral에서 약간 벗어난 경우(각도 오차 시뮬)
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
    dashed_ok = simulate_dashed_right_solid_selection()
    lidar_ok = simulate_lidar_distances()
    print("\n=== 결과 ===")
    print(f"마스크(바닥 덩어리 억제): {'PASS' if mask_ok else 'CHECK'}")
    print(f"점선-오른쪽 실선 선택: {'PASS' if dashed_ok else 'CHECK'}")
    print(f"라이다 150cm/50cm 감지: {'PASS' if lidar_ok else 'CHECK'}")
    if cfg.LIDAR_FRONT_ANGLE == 0.0:
        print(
            "\n주의: LIDAR_FRONT_ANGLE=0.0(미보정)이면 실차에서 멀수록 "
            "lateral이 어긋날 수 있습니다. calibrate_lidar_angle.py로 "
            "보정하세요."
        )


if __name__ == "__main__":
    main()
