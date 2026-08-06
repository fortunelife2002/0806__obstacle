"""라이다로 정면 좁은 시야각의 근거리 장애물을 감지하고, 감지 즉시 옆
차선으로 회피 주행하는 모듈입니다.

두 개의 클래스로 나뉩니다.

1) LidarObstacleDetector
   라이다 스캔은 시리얼로 계속 데이터가 들어오는 블로킹(제너레이터) 방식이라
   카메라 처리와 같은 스레드에서 돌리면 영상 루프가 멈춥니다. 그래서 별도
   스레드에서 계속 스캔하며, "정면 기준 좌우 N도, M mm 이내에 뭔가 있다/
   없다"라는 값 하나만 메인 스레드와 공유합니다.

2) AvoidanceController
   감지 결과를 받아 실제 회피 기동(옆 차선으로 정해진 시간 동안 이동)을
   관리하는 상태기계입니다. 카메라 차선 인식 결과를 잠깐 무시하고 오픈루프
   조향/속도로 옆 차선까지 이동한 뒤, lane_controller의 이전 경로 기억을
   지우고 일반 차선 추종으로 복귀시킵니다.
"""

import threading
import time

import Function_Library as fl

import config as cfg


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


class LidarObstacleDetector:
    """정면 좁은 시야각·근거리 장애물만 감지하는 백그라운드 라이다 스캐너."""

    def __init__(self):
        self._lidar = None
        self._lock = threading.Lock()
        self._detected = False
        self._consecutive_hits = 0
        self._last_scan_time = 0.0
        self._running = False
        self._thread = None

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

    def _angle_window(self):
        """0~360도 순환을 고려해 정면 기준 감지 각도 구간을 계산합니다."""
        low = (cfg.LIDAR_FRONT_ANGLE - cfg.LIDAR_DETECT_HALF_WINDOW_DEG) % 360.0
        high = (cfg.LIDAR_FRONT_ANGLE + cfg.LIDAR_DETECT_HALF_WINDOW_DEG) % 360.0
        return low, high

    def _hit_in_front_window(self, scan):
        """getAngleDistanceRange는 minAngle < maxAngle을 가정하므로, 감지
        구간이 0도를 넘어가는 경우(예: 358도~4도)는 두 구간으로 나눠 검사
        합니다."""
        low, high = self._angle_window()
        min_dist = cfg.LIDAR_DETECT_MIN_DISTANCE_MM
        max_dist = cfg.LIDAR_DETECT_MAX_DISTANCE_MM

        if low <= high:
            hit = self._lidar.getAngleDistanceRange(
                scan, low, high, min_dist, max_dist
            )
            return len(hit) > 0

        hit_a = self._lidar.getAngleDistanceRange(
            scan, low, 360.0, min_dist, max_dist
        )
        hit_b = self._lidar.getAngleDistanceRange(
            scan, 0.0, high, min_dist, max_dist
        )
        return len(hit_a) > 0 or len(hit_b) > 0

    def _scan_loop(self):
        try:
            for scan in self._lidar.scanning():
                if not self._running:
                    break
                if len(scan) == 0:
                    continue

                hit = self._hit_in_front_window(scan)
                with self._lock:
                    self._consecutive_hits = (
                        self._consecutive_hits + 1 if hit else 0
                    )
                    self._detected = (
                        self._consecutive_hits >= cfg.LIDAR_DETECT_CONFIRM_COUNT
                    )
                    self._last_scan_time = time.monotonic()
        except Exception as exc:
            print(f"LIDAR_SCAN_LOOP_ENDED: {exc}")

    def is_obstacle_detected(self):
        """정면 좁은 구간·근거리에 장애물이 있으면 True를 반환합니다.

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

    def stop(self):
        self._running = False
        if self._lidar is not None:
            try:
                self._lidar.stop()
            except Exception:
                pass


class AvoidanceController:
    """라이다 감지 결과로 옆 차선 회피 기동을 관리하는 상태기계."""

    IDLE = "IDLE"
    AVOIDING = "AVOIDING"
    COOLDOWN = "COOLDOWN"

    def __init__(self):
        self.state = self.IDLE
        self._state_until = 0.0

    def _avoid_command(self):
        target_steer = clamp(
            cfg.STEER_CENTER + cfg.AVOID_LANE_DIRECTION * cfg.AVOID_STEER_OFFSET,
            cfg.STEER_RIGHT,
            cfg.STEER_LEFT,
        )
        return {"speed": cfg.AVOID_SPEED, "steering": target_steer}

    def update(self, obstacle_detected, lane_command, lane_controller):
        """매 프레임 호출합니다.

        Returns:
            (command, reason): command는 {"speed", "steering"}, reason은
            디버그 표시/로그용 문자열입니다.
        """
        now = time.monotonic()

        if self.state == self.AVOIDING:
            if now >= self._state_until:
                self.state = self.COOLDOWN
                self._state_until = now + cfg.AVOID_COOLDOWN_SECONDS
                # 옆 차선으로 넘어온 뒤에는 이전 차선 기준의 경로 기억이
                # 오히려 방해가 되므로 지우고 새로 인식하게 합니다.
                lane_controller.reset_tracking()
                print("AVOID_DONE: 옆 차선 진입 완료, 일반 차선 추종으로 복귀합니다.")
            else:
                return self._avoid_command(), "AVOID"

        if self.state == self.COOLDOWN:
            if now >= self._state_until:
                self.state = self.IDLE
            else:
                return lane_command, "LANE_COOLDOWN"

        # 여기 도달하면 state == IDLE 입니다.
        if obstacle_detected:
            self.state = self.AVOIDING
            self._state_until = now + cfg.AVOID_DURATION_SECONDS
            print(
                "AVOID_START: 전방 "
                f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm 이내 장애물 감지, "
                "옆 차선으로 회피합니다."
            )
            return self._avoid_command(), "AVOID"

        return lane_command, "LANE"
