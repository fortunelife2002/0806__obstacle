"""라이다로 "지금 있는 차선" 안의 근거리 장애물만 감지하고, 감지될 때마다
옆 차선으로 목표를 토글하는 모듈입니다.

두 개의 클래스로 나뉩니다.

1) LidarObstacleDetector
   라이다 스캔은 시리얼로 계속 데이터가 들어오는 블로킹(제너레이터) 방식이라
   카메라 처리와 같은 스레드에서 돌리면 영상 루프가 멈춥니다. 그래서 별도
   스레드에서 계속 스캔하며, "내 차선 안, M mm 이내에 뭔가 있다/없다"라는
   값 하나만 메인 스레드와 공유합니다.

   트랙이 2차선이고 우리는 바깥쪽 2차선만 달리므로, 옆 1차선에 있는
   장애물까지 잡으면 안 됩니다. 정면 기준 "각도창"만으로는 두 차선이
   원근 때문에 거리가 멀어질수록 각도차가 좁아져(소실점 효과) 구분이
   안 되므로, 라이다 점을 차량 진행축 기준 직교좌표로 바꿔 판정합니다
   (_hit_in_lane_corridor). 실제 장애물은 config의 LIDAR_OBSTACLE_TRACK_*
   (기본 40~64cm ± LIDAR_OBSTACLE_CORRIDOR_EXPAND_CM) 트랙 좌표 구간
   구간에 놓이므로, lateral(mm)로 환산한 그 범위 안인지로 판정해 사람 등
   다른 물체를 장애물로 오검출하는 걸 막습니다.

2) AvoidanceController
   감지 결과를 받아 실제 회피 기동을 관리하는 상태기계입니다.

   [오픈루프 회피] 장애물 상승 에지에서 옆 차선으로 토글한 뒤,
   AVOID_OPEN_LOOP_OUT_SECONDS 동안 최대 조향 →
   AVOID_OPEN_LOOP_COUNTER_SECONDS 동안 반대 최대 조향으로 자세를 맞추고,
   이후 카메라 차선 추종으로 넘깁니다. 기동 중 속도는 AVOID_OPEN_LOOP_SPEED
   (기본 120)로 고정합니다.
"""

import threading
import time

import numpy as np

import Function_Library as fl

import config as cfg


class LidarObstacleDetector:
    """내 차선(폭 코리더) 안·근거리 장애물만 감지하는 백그라운드 라이다 스캐너."""

    def __init__(self):
        self._lidar = None
        self._lock = threading.Lock()
        self._detected = False
        self._consecutive_hits = 0
        self._last_scan_time = 0.0
        self._running = False
        self._thread = None
        # 실차에서 LIDAR_FRONT_ANGLE/LIDAR_OBSTACLE_LATERAL_*_MM이 맞는지 눈으로
        # 확인할 수 있도록, 매 스캔에서 가장 가까운 점의 lateral/forward/
        # in_lane 여부를 따로 기억해둡니다(get_debug_info 참고).
        self._nearest_debug = None
        self._lane_offset_lanes = 0.0

        if cfg.LIDAR_ENABLED:
            self._start()
        else:
            print("LIDAR_DISABLED: config.LIDAR_ENABLED = False")

    def _start(self):
        try:
            self._lidar = fl.libLIDAR(cfg.LIDAR_PORT)
            self._lidar.init()
        except Exception as exc:
            print(f"LIDAR_INIT_FAILED: {exc}")
            self._lidar = None
            return

        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        print(f"LIDAR_STARTED: port={cfg.LIDAR_PORT}")

    def _lane_geometry(self, scan):
        """라이다 점을 차량 진행축 기준 직교좌표로 바꿉니다.

        각도(도) 기준 극좌표를, 라이다 정면(cfg.LIDAR_FRONT_ANGLE) 방향을
        0으로 하는 직교좌표로 바꿔서 lateral(횡방향)/forward(전방) 성분을
        구합니다. 각도창 방식과 달리 거리에 상관없이 항상 "내 차선 폭"
        만큼만 보므로, 원근으로 옆 차선과의 각도차가 좁아지는 먼 거리에서도
        옆 차선(1차선) 장애물을 오검출하지 않습니다.

        라이다가 차체 중앙에 좌우 오프셋 없이 고정돼 있다는 전제로,
        차선 중심선이 곧 라이다 원점을 지난다고 가정해 ±(차선 폭/2)로
        대칭 검사합니다. 라이다가 중앙에서 벗어나 있다면 lateral 계산에
        오프셋 보정을 추가해야 합니다.

        Returns:
            (distance, lateral, forward) numpy 배열 튜플, 또는 scan이
            비어있으면 None.
        """
        data = np.asarray(scan, dtype=float)
        if data.size == 0:
            return None

        angle_diff_deg = (data[:, 0] - cfg.LIDAR_FRONT_ANGLE + 180.0) % 360.0 - 180.0
        angle_diff_rad = np.radians(angle_diff_deg)
        distance = data[:, 1]
        lateral = distance * np.sin(angle_diff_rad)
        forward = distance * np.cos(angle_diff_rad)
        return distance, lateral, forward

    def set_tracking_lane_offset(self, offset_lanes):
        """회피 중 감지 코리더를 옆 차선 기준으로 옮깁니다(0=2차선)."""
        self._lane_offset_lanes = float(offset_lanes)

    def _corridor_lateral_bounds(self, distance_mm):
        """LIDAR_OBSTACLE_TRACK_* 구간 + 회피 오프셋 + 거리 보정 slack."""
        lane_shift_mm = (
            self._lane_offset_lanes
            * cfg.AVOID_LANE_DIRECTION
            * cfg.LIDAR_LANE_WIDTH_MM
        )
        slack = distance_mm * cfg.LIDAR_LATERAL_DISTANCE_SLACK_RATIO
        lateral_min = (
            cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM - lane_shift_mm - slack
        )
        lateral_max = (
            cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM - lane_shift_mm + slack
        )
        return lateral_min, lateral_max

    def _hit_in_lane_corridor(self, scan):
        """차량 진행축 기준 "장애물 예상 구간"(LIDAR_OBSTACLE_TRACK_*) 이내·전방
        감지거리 이내에 점이 하나라도 있으면 (True, 디버그정보)를 반환합니다.

        차선 전체 폭(±170mm)이 아니라 실제 장애물이 놓이는 구간만 보는
        이유는, 트랙 주변에 있을 수 있는 사람 등 다른 물체를 장애물로 잘못
        잡지 않기 위해서입니다.

        디버그정보는 감지거리 범위 안에서 가장 가까운 점의 lateral/
        forward/in_lane 여부입니다(실차에서 LIDAR_FRONT_ANGLE/
        LIDAR_OBSTACLE_LATERAL_*_MM이 맞는지 화면으로 확인하는 용도이며,
        hit 판정 자체에는 쓰이지 않습니다). 범위 안에 점이 하나도 없으면
        None입니다.
        """
        geometry = self._lane_geometry(scan)
        if geometry is None:
            return False, None
        distance, lateral, forward = geometry

        in_range = (
            (forward > 0.0)
            & (distance >= cfg.LIDAR_DETECT_MIN_DISTANCE_MM)
            & (distance <= cfg.LIDAR_DETECT_MAX_DISTANCE_MM)
        )
        lateral_min, lateral_max = self._corridor_lateral_bounds(distance)
        in_lane = (
            (lateral >= lateral_min)
            & (lateral <= lateral_max)
        )
        hit = bool(np.any(in_range & in_lane))

        debug = None
        if np.any(in_range):
            nearest_idx = np.where(in_range)[0][np.argmin(distance[in_range])]
            debug = {
                "distance": float(distance[nearest_idx]),
                "lateral": float(lateral[nearest_idx]),
                "forward": float(forward[nearest_idx]),
                "in_lane": bool(in_lane[nearest_idx]),
            }
        return hit, debug

    def _scan_loop(self):
        try:
            for scan in self._lidar.scanning():
                if not self._running:
                    break
                if len(scan) == 0:
                    continue

                hit, debug = self._hit_in_lane_corridor(scan)
                with self._lock:
                    self._consecutive_hits = (
                        self._consecutive_hits + 1 if hit else 0
                    )
                    self._detected = (
                        self._consecutive_hits >= cfg.LIDAR_DETECT_CONFIRM_COUNT
                    )
                    self._last_scan_time = time.monotonic()
                    self._nearest_debug = debug
        except Exception as exc:
            print(f"LIDAR_SCAN_LOOP_ENDED: {exc}")

    def is_obstacle_detected(self):
        """내 차선 안·근거리에 장애물이 있으면 True를 반환합니다.

        라이다가 비활성/초기화 실패/통신 끊김 상태이면 항상 False를 반환해,
        센서 문제가 있어도 차량이 무한정 멈추지 않고 일반 차선 추종을
        계속하도록 합니다(정지가 필요하면 별도 초음파/타임아웃 로직으로
        처리해야 합니다).
        """
        if not cfg.LIDAR_ENABLED or self._lidar is None:
            return False
        with self._lock:
            if time.monotonic() - self._last_scan_time > cfg.LIDAR_STALE_SECONDS:
                return False
            return self._detected

    def get_debug_info(self):
        """가장 최근 스캔에서 감지거리 범위 안 가장 가까운 점의
        {"distance", "lateral", "forward", "in_lane"}을 반환합니다.

        LIDAR_FRONT_ANGLE(정면 각도)과 LIDAR_OBSTACLE_LATERAL_MIN/MAX_MM
        (장애물 예상 구간)이 실차에서 맞게 설정됐는지 화면으로 확인하는
        용도입니다: 예를 들어 실제 장애물을 놓을 위치에 물체를 놓고
        lateral이 LIDAR_OBSTACLE_LATERAL_MIN/MAX_MM 범위 안에서 in_lane이
        True로 나오는지 보면 됩니다. 범위 안에 점이 없거나 라이다가
        비활성/통신 끊김이면
        None을 반환합니다.
        """
        if not cfg.LIDAR_ENABLED or self._lidar is None:
            return None
        with self._lock:
            if time.monotonic() - self._last_scan_time > cfg.LIDAR_STALE_SECONDS:
                return None
            return dict(self._nearest_debug) if self._nearest_debug else None

    def stop(self):
        self._running = False
        if self._lidar is not None:
            try:
                self._lidar.stop()
            except Exception:
                pass


class AvoidanceController:
    """라이다 장애물 감지 시 오픈루프 차선 변경 후 카메라 추종으로 넘깁니다.

    begin_frame / finalize_frame 호출 순서는 main.py 주석을 참고하세요.
    """

    def __init__(self):
        self._current_offset = 0.0
        self._was_detected = False
        self._toggle_hold_until = 0.0
        self._maneuver_phase = None
        self._phase_until = 0.0
        self._maneuver_target_offset = 0.0

    @property
    def state(self):
        """디버그 표시/로그용."""
        if self._maneuver_phase == "OUT":
            return "AVOID_OUT"
        if self._maneuver_phase == "COUNTER":
            return "AVOID_COUNTER"
        if self._current_offset != 0.0:
            return "AVOIDING"
        return "IDLE"

    @property
    def lane_offset_lanes(self):
        return self._current_offset

    @property
    def maneuver_phase(self):
        return self._maneuver_phase

    @property
    def in_open_loop(self):
        return self._maneuver_phase is not None

    @staticmethod
    def _open_loop_duration():
        if not cfg.AVOID_OPEN_LOOP_ENABLED:
            return 0.0
        return (
            cfg.AVOID_OPEN_LOOP_OUT_SECONDS
            + cfg.AVOID_OPEN_LOOP_COUNTER_SECONDS
        )

    def _moving_to_left_lane(self):
        return self._maneuver_target_offset != 0.0

    def _out_steering(self):
        if self._moving_to_left_lane():
            return cfg.AVOID_OPEN_LOOP_STEER_LEFT
        return cfg.AVOID_OPEN_LOOP_STEER_RIGHT

    def _counter_steering(self):
        if self._moving_to_left_lane():
            return cfg.AVOID_OPEN_LOOP_STEER_RIGHT
        return cfg.AVOID_OPEN_LOOP_STEER_LEFT

    def _start_open_loop_maneuver(self, now, target_offset):
        self._maneuver_target_offset = float(target_offset)
        self._maneuver_phase = "OUT"
        self._phase_until = now + cfg.AVOID_OPEN_LOOP_OUT_SECONDS
        direction = "왼쪽(1차선)" if self._moving_to_left_lane() else "오른쪽(2차선)"
        print(
            "AVOID_OPEN_LOOP: "
            f"{direction}으로 이동 시작 "
            f"(OUT {cfg.AVOID_OPEN_LOOP_OUT_SECONDS:.1f}s "
            f"STR={self._out_steering()}, "
            f"SPD={cfg.AVOID_OPEN_LOOP_SPEED})"
        )

    def _advance_open_loop_maneuver(self, now, lane_controller=None):
        if self._maneuver_phase is None:
            return
        if now < self._phase_until:
            return
        if self._maneuver_phase == "OUT":
            self._maneuver_phase = "COUNTER"
            self._phase_until = now + cfg.AVOID_OPEN_LOOP_COUNTER_SECONDS
            print(
                "AVOID_OPEN_LOOP: 반대 조향 보정 "
                f"(COUNTER {cfg.AVOID_OPEN_LOOP_COUNTER_SECONDS:.1f}s "
                f"STR={self._counter_steering()}, "
                f"SPD={cfg.AVOID_OPEN_LOOP_SPEED})"
            )
            return
        if self._maneuver_phase == "COUNTER":
            self._maneuver_phase = None
            self._phase_until = 0.0
            print("AVOID_OPEN_LOOP: 기동 완료, 카메라 차선 추종으로 전환")

    def _sync_lane_offset(self, lane_controller):
        """오픈루프가 아닐 때 lane_controller 오프셋을 회피 목표와 맞춥니다."""
        if lane_controller is None:
            return
        if self._maneuver_phase is None:
            lane_controller.set_lane_offset(self._current_offset)
        elif cfg.AVOID_OPEN_LOOP_ENABLED:
            lane_controller.set_lane_offset(0.0)

    def begin_frame(self, obstacle_detected, lane_controller):
        """카메라 처리 전에 매 프레임 호출합니다."""
        now = time.monotonic()
        self._advance_open_loop_maneuver(now, lane_controller)
        self._sync_lane_offset(lane_controller)

        if obstacle_detected and not self._was_detected:
            if (
                now >= self._toggle_hold_until
                and self._maneuver_phase is None
            ):
                new_offset = (
                    0.0
                    if self._current_offset != 0.0
                    else cfg.AVOID_LANE_OFFSET_LANES
                )
                self._current_offset = new_offset
                maneuver_duration = self._open_loop_duration()
                hold_seconds = max(
                    cfg.AVOID_TOGGLE_HOLD_SECONDS,
                    maneuver_duration + 0.5,
                )
                self._toggle_hold_until = now + hold_seconds

                if cfg.AVOID_OPEN_LOOP_ENABLED:
                    self._start_open_loop_maneuver(now, new_offset)
                    lane_controller.set_lane_offset(0.0)
                else:
                    lane_controller.set_lane_offset(self._current_offset)
                    target = "옆" if self._current_offset != 0.0 else "원래"
                    print(
                        "AVOID_TOGGLE: 내 차선 전방 "
                        f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm 이내 장애물 감지, "
                        f"{target} 차선을 목표로 옮깁니다(카메라가 계속 보정)."
                    )
        self._was_detected = obstacle_detected

    def finalize_frame(self, lane_command, lane_controller=None):
        """카메라 처리 후 매 프레임 호출합니다."""
        now = time.monotonic()
        self._advance_open_loop_maneuver(now, lane_controller)
        self._sync_lane_offset(lane_controller)

        if self._maneuver_phase == "OUT":
            return {
                "speed": cfg.AVOID_OPEN_LOOP_SPEED,
                "steering": self._out_steering(),
            }, "AVOID_OUT"

        if self._maneuver_phase == "COUNTER":
            return {
                "speed": cfg.AVOID_OPEN_LOOP_SPEED,
                "steering": self._counter_steering(),
            }, "AVOID_COUNTER"

        if self._current_offset != 0.0:
            command = dict(lane_command)
            command["speed"] = min(command["speed"], cfg.AVOID_SPEED)
            return command, "AVOIDING"

        return lane_command, "LANE"
