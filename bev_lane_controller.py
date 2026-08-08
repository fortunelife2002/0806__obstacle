"""BEV(조감도) 기반 차선 추종 제어기.

Phase1-43.py의 BEV 파이프라인(고정 임계값 이진화 + 호모그래피)을 사용합니다.
밝기 적응 노출 대신 고정 THRESHOLD로 차선만 추출합니다.

차선 규칙
- 점선이 화면 왼쪽 → 2차선: 점선 바로 오른쪽 실선을 우측 차선
- 점선이 화면 오른쪽 → 1차선: 점선 바로 왼쪽 실선을 좌측 차선
- 장애물 회피 오프셋(lanes!=0)이면 1차선 모드로 고정
"""

import json
import math
import os
import time

import cv2
import numpy as np

import config as cfg


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def load_calibration():
    """BEV 호모그래피와 차량 기준점을 만든다."""
    source_points = list(cfg.BEV_SOURCE_POINTS)
    lane_width_cm = cfg.BEV_MARKER_WIDTH_CM
    length_cm = cfg.BEV_MARKER_LENGTH_CM
    pixels_per_cm = cfg.BEV_PIXELS_PER_CM
    bev_width = cfg.BEV_WIDTH
    bev_height = cfg.BEV_HEIGHT
    origin = "config 내장값"

    if os.path.exists(cfg.BEV_CALIB_FILE):
        try:
            with open(cfg.BEV_CALIB_FILE, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            source_points = [tuple(point) for point in data["source_points"]]
            lane_width_cm = float(data["lane_width_cm"])
            length_cm = float(data["marker_length_cm"])
            pixels_per_cm = float(data["pixels_per_cm"])
            bev_width = int(data["bev_width"])
            bev_height = int(data["bev_height"])
            origin = cfg.BEV_CALIB_FILE
        except OSError as error:
            print(f"경고: {cfg.BEV_CALIB_FILE} 읽기 실패, 내장값 사용 ({error})")

    half_width = lane_width_cm * pixels_per_cm / 2.0
    length_px = length_cm * pixels_per_cm
    centre_x = bev_width / 2.0
    near_y = bev_height - cfg.BEV_NEAR_MARGIN_PX
    far_y = near_y - length_px

    destination = np.float32([
        [centre_x - half_width, near_y],
        [centre_x + half_width, near_y],
        [centre_x + half_width, far_y],
        [centre_x - half_width, far_y],
    ])
    transform = cv2.getPerspectiveTransform(
        np.float32(source_points), destination
    )

    roi_height = cfg.ROI_BOTTOM - cfg.ROI_TOP
    vehicle = cv2.perspectiveTransform(
        np.float32([[[cfg.REFERENCE_WIDTH / 2.0, roi_height - 1]]]),
        transform,
    )
    return {
        "transform": transform,
        "pixels_per_cm": pixels_per_cm,
        "bev_width": bev_width,
        "bev_height": bev_height,
        "vehicle_x": float(vehicle[0][0][0]),
        "vehicle_y": float(vehicle[0][0][1]),
        "origin": origin,
    }


def preprocess_roi(roi):
    """고정 임계값으로 흰 차선 이진화."""
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, cfg.BEV_GAUSSIAN_KERNEL, 0)
    _, binary = cv2.threshold(
        blurred, cfg.BEV_THRESHOLD_VALUE, 255, cv2.THRESH_BINARY
    )
    return binary


def remove_horizontal_markings(binary_image):
    """정지선·횡단보도 가로 성분 제거."""
    horizontal_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (cfg.BEV_HORIZONTAL_KERNEL_WIDTH, cfg.BEV_HORIZONTAL_KERNEL_HEIGHT),
    )
    horizontal_mask = cv2.morphologyEx(
        binary_image, cv2.MORPH_OPEN, horizontal_kernel
    )
    lane_binary = cv2.bitwise_and(
        binary_image, cv2.bitwise_not(horizontal_mask)
    )
    vertical_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (3, cfg.BEV_VERTICAL_RECONNECT_HEIGHT)
    )
    lane_binary = cv2.morphologyEx(
        lane_binary, cv2.MORPH_CLOSE, vertical_kernel
    )
    return lane_binary, horizontal_mask


def warp_to_bev(image, calibration):
    return cv2.warpPerspective(
        image,
        calibration["transform"],
        (calibration["bev_width"], calibration["bev_height"]),
        flags=cv2.INTER_LINEAR,
    )


def find_white_runs(binary_image, center_y):
    """한 스캔 높이에서 흰 세로선 중심 x 목록."""
    height, _ = binary_image.shape
    y_start = max(0, int(center_y) - cfg.BEV_SCAN_HALF_HEIGHT_PX)
    y_end = min(height, int(center_y) + cfg.BEV_SCAN_HALF_HEIGHT_PX + 1)
    if y_end <= y_start:
        return []

    band = binary_image[y_start:y_end]
    needed = max(2, int(math.ceil(band.shape[0] * 0.5)))
    active = np.count_nonzero(band, axis=0) >= needed

    runs = []
    start = None
    for x, is_active in enumerate(active):
        if is_active and start is None:
            start = x
        last = x == len(active) - 1
        if start is not None and (not is_active or last):
            end = x if (is_active and last) else x - 1
            run_width = end - start + 1
            if cfg.BEV_MIN_LINE_WIDTH_PX <= run_width <= cfg.BEV_MAX_LINE_WIDTH_PX:
                runs.append((start + end) / 2.0)
            start = None
    return runs


def _assign_cluster(clusters, x, y, window):
    for cluster in clusters:
        if abs(cluster["last_x"] - x) <= window:
            cluster["points"].append((y, x))
            cluster["last_x"] = x
            return
    clusters.append({"points": [(y, x)], "last_x": x})


def _cluster_lines(bev_binary):
    """스캔 행마다 선을 묶어 점선/실선 후보 클러스터를 만든다."""
    clusters = []
    scan_rows = list(range(
        cfg.BEV_SCAN_BOTTOM_PX,
        cfg.BEV_SCAN_TOP_PX,
        -cfg.BEV_SCAN_STEP_PX,
    ))
    for y in scan_rows:
        for x in find_white_runs(bev_binary, y):
            _assign_cluster(clusters, x, y, cfg.BEV_TRACK_WINDOW_PX)

    total_rows = max(1, len(scan_rows))
    for cluster in clusters:
        xs = [point[1] for point in cluster["points"]]
        cluster["mean_x"] = float(np.mean(xs))
        cluster["continuity"] = len(cluster["points"]) / total_rows
    return clusters, scan_rows


def _resolve_lane_mode(dashed_x, bev_width, forced_offset):
    if forced_offset != 0.0:
        return 1
    if dashed_x < bev_width * 0.5:
        return 2
    return 1


def _pick_adjacent_solid(runs, dashed_x, lane_mode):
    margin = cfg.BEV_ADJACENT_MARGIN_PX
    if lane_mode == 2:
        right = [x for x in runs if x > dashed_x + margin]
        return min(right) if right else None
    left = [x for x in runs if x < dashed_x - margin]
    return max(left) if left else None


def _nearest_cluster_x(clusters, x, window):
    candidates = [
        cluster for cluster in clusters
        if abs(cluster["mean_x"] - x) <= window
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda item: abs(item["mean_x"] - x))


def detect_lane_center_points(bev_binary, calibration, forced_offset, previous):
    """점선 위치로 차선 모드를 정하고 차로 중앙점을 수집한다."""
    clusters, scan_rows = _cluster_lines(bev_binary)
    if not clusters:
        return None

    width = calibration["bev_width"]
    track_clusters = [
        cluster for cluster in clusters
        if cfg.BEV_TRACK_MIN_X <= cluster["mean_x"] <= cfg.BEV_TRACK_MAX_X
    ]
    if not track_clusters:
        track_clusters = clusters

    dashed_cluster = min(
        track_clusters,
        key=lambda cluster: cluster["continuity"],
    )
    dashed_x = dashed_cluster["mean_x"]
    lane_mode = _resolve_lane_mode(dashed_x, width, forced_offset)

    center_points = []
    previous_dashed = previous.get("dashed_x") if previous else None
    previous_solid = previous.get("solid_x") if previous else None

    for y in scan_rows:
        runs = find_white_runs(bev_binary, y)
        if not runs:
            continue

        if previous_dashed is not None:
            dashed_candidates = [
                x for x in runs
                if abs(x - previous_dashed) <= cfg.BEV_STEP_JUMP_PX
            ]
            dashed_x_row = (
                min(dashed_candidates, key=lambda x: abs(x - previous_dashed))
                if dashed_candidates
                else None
            )
        else:
            dashed_candidates = [
                x for x in runs
                if abs(x - dashed_x) <= cfg.BEV_TRACK_WINDOW_PX
            ]
            dashed_x_row = (
                min(dashed_candidates, key=lambda x: abs(x - dashed_x))
                if dashed_candidates
                else None
            )

        if dashed_x_row is None:
            continue

        solid_x_row = _pick_adjacent_solid(runs, dashed_x_row, lane_mode)
        if solid_x_row is None and previous_solid is not None:
            solid_candidates = [
                x for x in runs
                if abs(x - previous_solid) <= cfg.BEV_STEP_JUMP_PX
            ]
            if solid_candidates:
                if lane_mode == 2:
                    solid_x_row = min(
                        solid_candidates,
                        key=lambda x: abs(x - previous_solid),
                    )
                else:
                    solid_x_row = min(
                        solid_candidates,
                        key=lambda x: abs(x - previous_solid),
                    )

        if solid_x_row is None:
            continue

        center_x = (dashed_x_row + solid_x_row) * 0.5
        center_points.append({
            "y": float(y),
            "center_x": center_x,
            "dashed_x": dashed_x_row,
            "solid_x": solid_x_row,
        })
        previous_dashed = dashed_x_row
        previous_solid = solid_x_row

    if len(center_points) < cfg.BEV_MIN_CENTER_POINTS:
        return None

    return {
        "lane_mode": lane_mode,
        "dashed_x": dashed_x,
        "points": center_points,
        "last_dashed_x": previous_dashed,
        "last_solid_x": previous_solid,
    }


def fit_center_geometry(center_info, calibration):
    """차로 중앙점을 미터 단위 2차식으로 맞춘다."""
    points = center_info["points"]
    if len(points) < cfg.BEV_MIN_CENTER_POINTS:
        return None

    pixels_per_cm = calibration["pixels_per_cm"]
    vehicle_x = calibration["vehicle_x"]
    vehicle_y = calibration["vehicle_y"]

    forward = np.array(
        [(vehicle_y - item["y"]) / pixels_per_cm for item in points],
        dtype=np.float64,
    )
    lateral = np.array(
        [(item["center_x"] - vehicle_x) / pixels_per_cm for item in points],
        dtype=np.float64,
    )
    span_cm = float(np.max(forward) - np.min(forward))
    if span_cm < cfg.BEV_MIN_FIT_SPAN_CM:
        return None

    poly = np.polyfit(forward, lateral, 2)
    residual = lateral - np.polyval(poly, forward)
    keep = np.abs(residual) <= cfg.BEV_FIT_OUTLIER_CM
    if int(np.count_nonzero(keep)) >= cfg.BEV_MIN_CENTER_POINTS:
        poly = np.polyfit(forward[keep], lateral[keep], 2)
        residual = lateral[keep] - np.polyval(poly, forward[keep])

    max_residual = float(np.max(np.abs(residual)))
    if max_residual > cfg.BEV_FIT_MAX_RESIDUAL_CM:
        return None

    slope = float(poly[1])
    curvature = float(2.0 * poly[0] / math.pow(1.0 + slope * slope, 1.5))
    curvature = clamp(
        curvature, -cfg.BEV_CURVATURE_LIMIT_ABS, cfg.BEV_CURVATURE_LIMIT_ABS
    )
    return {
        "poly": poly,
        "lateral_cm": float(poly[2]),
        "heading_deg": math.degrees(math.atan(slope)),
        "curvature": curvature,
        "point_count": len(points),
        "span_cm": span_cm,
        "lane_mode": center_info["lane_mode"],
        "dashed_x": center_info["dashed_x"],
    }


def calculate_steering(geometry, steer_state, delta_time):
    """BEV 차로 중앙 오차로 조향을 계산한다."""
    lateral_error = geometry["lateral_cm"]
    heading_error = geometry["heading_deg"]
    curvature = geometry["curvature"]

    feedforward = clamp(
        cfg.BEV_FF_COUNTS_PER_CURVATURE * (-curvature),
        -cfg.BEV_FF_LIMIT,
        cfg.BEV_FF_LIMIT,
    )
    cross_track = clamp(
        cfg.BEV_KE_COUNTS_PER_CM * (-lateral_error),
        -cfg.BEV_CTE_LIMIT,
        cfg.BEV_CTE_LIMIT,
    )
    heading_term = clamp(
        cfg.BEV_KPSI_COUNTS_PER_DEG * (-heading_error),
        -cfg.BEV_PSI_LIMIT,
        cfg.BEV_PSI_LIMIT,
    )
    total = feedforward + cross_track + heading_term

    target = cfg.STEER_CENTER + total
    max_delta = cfg.BEV_STEER_RATE_PER_SECOND * delta_time
    steer_state["value"] = clamp(
        target,
        steer_state["value"] - max_delta,
        steer_state["value"] + max_delta,
    )
    steering = int(round(clamp(
        steer_state["value"],
        cfg.STEER_RIGHT,
        cfg.STEER_LEFT,
    )))
    return steering, lateral_error, heading_error, curvature


class BevLaneController:
    """BEV 기반 차선 추종 제어기."""

    def __init__(self):
        self.calibration = load_calibration()
        self.steer_state = {"value": float(cfg.STEER_CENTER)}
        self.track_memory = None
        self.lost_count = 0
        self.last_update_time = None
        self._lane_offset_lanes = 0.0
        self.last_geometry = None
        print(
            "BEV 캘리브레이션: "
            f"{self.calibration['origin']}, "
            f"차량=({self.calibration['vehicle_x']:.1f}, "
            f"{self.calibration['vehicle_y']:.1f})"
        )

    def set_lane_offset(self, lanes):
        """회피 중 목표 차선. 0=2차선, 0이 아니면 1차선 모드."""
        self._lane_offset_lanes = float(lanes)

    def reset_tracking(self):
        self.track_memory = None
        self.lost_count = 0
        self.last_geometry = None
        self.steer_state["value"] = float(cfg.STEER_CENTER)

    def update(self, frame):
        now = time.monotonic()
        if self.last_update_time is None:
            delta_time = 1.0 / 20.0
        else:
            delta_time = clamp(now - self.last_update_time, 0.01, 0.12)
        self.last_update_time = now

        bottom = min(cfg.ROI_BOTTOM, frame.shape[0])
        top = min(cfg.ROI_TOP, bottom - 1)
        roi = frame[top:bottom]
        lane_binary = preprocess_roi(roi)
        lane_binary, _ = remove_horizontal_markings(lane_binary)
        bev_binary = warp_to_bev(lane_binary, self.calibration)
        bev_colour = warp_to_bev(roi, self.calibration)

        center_info = detect_lane_center_points(
            bev_binary,
            self.calibration,
            self._lane_offset_lanes,
            self.track_memory,
        )
        geometry = (
            fit_center_geometry(center_info, self.calibration)
            if center_info is not None
            else None
        )

        if geometry is not None:
            self.lost_count = 0
            self.last_geometry = geometry
            self.track_memory = {
                "dashed_x": center_info["last_dashed_x"],
                "solid_x": center_info["last_solid_x"],
            }
            steering, lateral_error, heading_error, curvature = (
                calculate_steering(geometry, self.steer_state, delta_time)
            )
            status = "TRACKING"
            confidence = min(
                1.0,
                geometry["point_count"] / max(1.0, cfg.BEV_MIN_CENTER_POINTS * 1.5),
            )
            speed = cfg.BASE_SPEED
            lane_mode = geometry["lane_mode"]
        else:
            self.lost_count += 1
            if self.lost_count <= cfg.BEV_MEMORY_FRAMES and self.last_geometry:
                geometry = self.last_geometry
                steering, lateral_error, heading_error, curvature = (
                    calculate_steering(geometry, self.steer_state, delta_time)
                )
                status = "MEMORY"
                confidence = 0.0
                speed = cfg.RETURN_SPEED
                lane_mode = geometry["lane_mode"]
            else:
                steering = cfg.STEER_CENTER
                lateral_error = 0.0
                heading_error = 0.0
                curvature = 0.0
                status = "LOST"
                confidence = 0.0
                speed = 0
                lane_mode = 2 if self._lane_offset_lanes == 0.0 else 1

        if lane_mode == 2:
            near_source = "DASH_L"
            far_source = "SOLID_R"
            single_side = "RIGHT"
        else:
            near_source = "SOLID_L"
            far_source = "DASH_R"
            single_side = "LEFT"

        return {
            "status": status,
            "speed": speed,
            "steering": steering,
            "confidence": confidence,
            "center_error": lateral_error * (cfg.REFERENCE_WIDTH / self.calibration["bev_width"]),
            "heading_error": heading_error,
            "curvature": curvature,
            "near_source": near_source,
            "far_source": far_source,
            "lane_offset_lanes": self._lane_offset_lanes,
            "offset_lane_width_px": 0.0,
            "left_boundary_side": single_side if status != "LOST" else None,
            "preview_error": 0.0,
            "preview_weight": 0.0,
            "guard": 0.0,
            "left_guard": 0.0,
            "hard_boundary": 0,
            "single_lane_side": single_side if status in ("TRACKING", "MEMORY") else None,
            "single_lane_frames": 0,
            "corridor_steer": 0.0,
            "lateral_safety": 0.0,
            "mask": bev_binary,
            "bev_colour": bev_colour,
            "geometry": geometry,
            "lane_mode": lane_mode,
        }

    @staticmethod
    def draw_debug(frame, lane_result):
        """BEV 디버그 정보를 주행 화면에 표시한다."""
        geometry = lane_result.get("geometry")
        if geometry is None:
            return frame
        text = (
            f"BEV L{lane_result.get('lane_mode', '-')}"
            f" D:{geometry.get('dashed_x', 0):.0f}"
            f" LAT:{geometry.get('lateral_cm', 0):.1f}cm"
        )
        cv2.putText(
            frame, text, (8, 164),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1,
        )
        return frame
