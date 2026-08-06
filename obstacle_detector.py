"""라이다로 "내 차선(2차선, 바깥쪽)" 안의 근거리 장애물만 감지하고, 감지
즉시 옆 1차선으로 회피했다가 장애물을 지나치면 다시 2차선으로 복귀하는
모듈입니다.

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
   (_hit_in_lane_corridor). 실제 장애물은 항상 차량 중앙(=우리 차선
   중앙으로 가정) 기준 좌우 50mm 이내에만 놓이므로, "횡방향 거리가 그
   범위 안인가"로 좁혀서 판정해 사람 등 다른 물체를 장애물로 오검출하는
   걸 막습니다.

2) AvoidanceController
   감지 결과를 받아 실제 회피 기동을 관리하는 상태기계입니다.

   [2026-08-06 Claude 수정 3] 예전에는 "정해진 각도로 정해진 시간만
   꺾는" 오픈루프(카메라 무시) 방식으로 OUT/HOLD/RETURN 세 구간을
   시간으로 관리했는데, 실차에서 각도/시간 조합이 너무 민감해
   (조금만 오차가 나도 과회전하거나 못 미치는 문제 반복) 계속
   재조정이 필요했습니다.

   지금은 조향을 직접 계산하지 않고, LaneController.set_lane_offset()
   으로 "목표 차선을 옆으로 옮겨라"라고만 지시합니다. 실제 조향은
   기존 카메라 차선 추종 PID가 매 프레임 계속 담당하므로, 옆 차선에
   정확히 안착할 때까지 카메라 피드백이 계속 작동합니다(각도/시간을
   따로 맞출 필요가 없어짐). 상태는 IDLE -> AVOIDING(오프셋 적용) ->
   COOLDOWN(오프셋 해제, 재감지 무시) -> IDLE로 단순해졌습니다.
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

    def _hit_in_lane_corridor(self, scan):
        """차량 진행축 기준 "장애물 예상 구간"(중앙에서 횡방향 0~50mm)
        이내·전방 감지거리 이내에 점이 하나라도 있으면 (True, 디버그정보)를
        반환합니다.

        차선 전체 폭(±170mm)이 아니라 실제 장애물이 놓이는 좁은 구간만
        보는 이유는, 트랙 주변에 있을 수 있는 사람 등 다른 물체를 장애물로
        잘못 잡지 않기 위해서입니다. 부호(좌/우)는 가리지 않고 중앙에서의
        절대 횡방향 거리만 보므로, 장애물이 차량 중앙 기준 어느 쪽으로
        살짝 치우쳐 있어도 잡힙니다.

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
        abs_lateral = np.abs(lateral)
        in_lane = (
            (abs_lateral >= cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM)
            & (abs_lateral <= cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM)
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
        lateral 절대값이 50mm 이내에서 in_lane이 True로 나오는지 보면
        됩니다. 범위 안에 점이 없거나 라이다가 비활성/통신 끊김이면
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
    """라이다 감지 결과로 카메라 기반 차선 오프셋 회피를 관리하는 상태기계.

    조향은 직접 계산하지 않습니다 — LaneController.set_lane_offset()으로
    목표 차선만 지시하고, 실제 조향은 매 프레임 카메라 차선 추종 PID가
    계속 담당합니다. 그래서 이 클래스는 두 단계로 나눠 호출해야 합니다.

      1) begin_frame(): lane_controller.update(frame) 하기 "전"에 호출.
         상태 전이를 결정하고 필요하면 오프셋을 설정합니다.
      2) finalize_frame(): lane_controller.update(frame) 하고 난 "후"에
         호출. 최종 speed/steering과 디버그용 reason을 반환합니다.
    """

    IDLE = "IDLE"
    AVOIDING = "AVOIDING"
    COOLDOWN = "COOLDOWN"

    def __init__(self):
        self.state = self.IDLE
        self._state_until = 0.0

    def begin_frame(self, obstacle_detected, lane_controller):
        """카메라 처리 전에 매 프레임 호출합니다. 상태 전이만 담당합니다.

        각 조건을 elif가 아닌 개별 if로 검사해서, 같은 프레임 안에서
        COOLDOWN -> IDLE -> (장애물 있으면) AVOIDING까지 연달아 전이될 수
        있게 합니다(쿨다운이 끝나는 바로 그 프레임에 장애물이 여전히
        있으면 한 프레임도 안 쉬고 바로 재감지하기 위함).
        """
        now = time.monotonic()

        if self.state == self.AVOIDING and now >= self._state_until:
            self.state = self.COOLDOWN
            self._state_until = now + cfg.AVOID_COOLDOWN_SECONDS
            lane_controller.set_lane_offset(0.0)
            print("AVOID_DONE: 2차선 목표로 복귀, 일반 차선 추종을 계속합니다.")

        if self.state == self.COOLDOWN and now >= self._state_until:
            self.state = self.IDLE

        if self.state == self.IDLE and obstacle_detected:
            self.state = self.AVOIDING
            self._state_until = now + cfg.AVOID_DURATION_SECONDS
            lane_controller.set_lane_offset(cfg.AVOID_LANE_OFFSET_LANES)
            print(
                "AVOID_START: 내 차선(2차선) 전방 "
                f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm 이내 장애물 감지, "
                "옆 차선을 목표로 옮깁니다(카메라가 계속 보정)."
            )

    def finalize_frame(self, lane_command):
        """카메라 처리 후 매 프레임 호출합니다.

        조향은 항상 lane_command(카메라 제어 결과, 이미 오프셋이 반영됨)를
        그대로 씁니다. 회피 중에는 안전하게 속도만 제한합니다.

        Returns:
            (command, reason): command는 {"speed", "steering", ...},
            reason은 디버그 표시/로그용 문자열입니다.
        """
        if self.state == self.AVOIDING:
            command = dict(lane_command)
            command["speed"] = min(command["speed"], cfg.AVOID_SPEED)
            return command, "AVOIDING"
        if self.state == self.COOLDOWN:
            return lane_command, "LANE_COOLDOWN"
        return lane_command, "LANE"
