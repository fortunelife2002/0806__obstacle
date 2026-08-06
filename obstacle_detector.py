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
   안 되므로, 라이다 점을 차량 진행축 기준 직교좌표로 바꿔 "횡방향
   거리가 내 차선 폭 절반 이내인가"로 판정합니다(_hit_in_lane_corridor).

2) AvoidanceController
   감지 결과를 받아 실제 회피 기동을 관리하는 상태기계입니다. 카메라
   차선 인식 결과를 잠깐 무시하고 오픈루프 조향/속도로 다음 세 구간을
   순서대로 실행합니다.

     OUT    : 1차선 방향으로 꺾어 차로를 변경합니다.
     HOLD   : 조향을 중앙으로 돌리고 직진하며 장애물 옆을 통과합니다.
     RETURN : OUT과 반대 방향으로 꺾어 다시 2차선으로 돌아옵니다.

   복귀가 끝나면 lane_controller의 이전 경로 기억을 지우고 일반 차선
   추종으로 되돌립니다. 카메라 인식은 "지금 보이는 오른쪽 선을 기준
   삼는" 상대적 방식이라, RETURN 없이 그냥 두면 1차선을 새 기준으로
   착각하고 눌러앉게 됩니다.
"""

import threading
import time

import numpy as np

import Function_Library as fl

import config as cfg


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


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

    def _hit_in_lane_corridor(self, scan):
        """차량 진행축 기준 좌우 내 차선 폭 이내·전방 감지거리 이내에
        점이 하나라도 있으면 True를 반환합니다.

        각도(도) 기준 극좌표를, 라이다 정면(cfg.LIDAR_FRONT_ANGLE) 방향을
        0으로 하는 직교좌표로 바꿔서 lateral(횡방향)/forward(전방) 성분을
        구합니다. 각도창 방식과 달리 거리에 상관없이 항상 "내 차선 폭"
        만큼만 보므로, 원근으로 옆 차선과의 각도차가 좁아지는 먼 거리에서도
        옆 차선(1차선) 장애물을 오검출하지 않습니다.

        라이다가 차체 중앙에 좌우 오프셋 없이 고정돼 있다는 전제로,
        차선 중심선이 곧 라이다 원점을 지난다고 가정해 ±(차선 폭/2)로
        대칭 검사합니다. 라이다가 중앙에서 벗어나 있다면 lateral 계산에
        오프셋 보정을 추가해야 합니다.
        """
        data = np.asarray(scan, dtype=float)
        if data.size == 0:
            return False

        angle_diff_deg = (data[:, 0] - cfg.LIDAR_FRONT_ANGLE + 180.0) % 360.0 - 180.0
        angle_diff_rad = np.radians(angle_diff_deg)
        distance = data[:, 1]
        lateral = distance * np.sin(angle_diff_rad)
        forward = distance * np.cos(angle_diff_rad)

        half_lane_width = cfg.LIDAR_LANE_WIDTH_MM * 0.5
        mask = (
            (forward > 0.0)
            & (np.abs(lateral) <= half_lane_width)
            & (distance >= cfg.LIDAR_DETECT_MIN_DISTANCE_MM)
            & (distance <= cfg.LIDAR_DETECT_MAX_DISTANCE_MM)
        )
        return bool(np.any(mask))

    def _scan_loop(self):
        try:
            for scan in self._lidar.scanning():
                if not self._running:
                    break
                if len(scan) == 0:
                    continue

                hit = self._hit_in_lane_corridor(scan)
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

    def stop(self):
        self._running = False
        if self._lidar is not None:
            try:
                self._lidar.stop()
            except Exception:
                pass


class AvoidanceController:
    """라이다 감지 결과로 "1차선 회피 -> 2차선 복귀" 기동을 관리하는 상태기계."""

    IDLE = "IDLE"
    AVOID_OUT = "AVOID_OUT"
    AVOID_HOLD = "AVOID_HOLD"
    AVOID_RETURN = "AVOID_RETURN"
    COOLDOWN = "COOLDOWN"

    def __init__(self):
        self.state = self.IDLE
        self._state_until = 0.0

    def _steer_command(self, direction_sign):
        """direction_sign: +1이면 AVOID_LANE_DIRECTION 쪽(1차선)으로,
        -1이면 그 반대쪽(2차선 복귀 방향)으로, 0이면 중앙(직진)으로
        꺾은 명령을 반환합니다."""
        target_steer = clamp(
            cfg.STEER_CENTER
            + direction_sign * cfg.AVOID_LANE_DIRECTION * cfg.AVOID_STEER_OFFSET,
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

        if self.state == self.AVOID_OUT:
            if now >= self._state_until:
                self.state = self.AVOID_HOLD
                self._state_until = now + cfg.AVOID_HOLD_SECONDS
                print("AVOID_HOLD: 1차선 진입 완료, 장애물 옆을 통과합니다.")
            else:
                return self._steer_command(+1), "AVOID_OUT"

        if self.state == self.AVOID_HOLD:
            if now >= self._state_until:
                self.state = self.AVOID_RETURN
                self._state_until = now + cfg.AVOID_RETURN_SECONDS
                print("AVOID_RETURN: 장애물 통과 완료, 2차선으로 복귀합니다.")
            else:
                return self._steer_command(0), "AVOID_HOLD"

        if self.state == self.AVOID_RETURN:
            if now >= self._state_until:
                self.state = self.COOLDOWN
                self._state_until = now + cfg.AVOID_COOLDOWN_SECONDS
                # 2차선으로 돌아온 뒤에는 1차선 기준의 경로 기억이 오히려
                # 방해가 되므로 지우고 새로 인식하게 합니다.
                lane_controller.reset_tracking()
                print("AVOID_DONE: 2차선 복귀 완료, 일반 차선 추종으로 복귀합니다.")
            else:
                return self._steer_command(-1), "AVOID_RETURN"

        if self.state == self.COOLDOWN:
            if now >= self._state_until:
                self.state = self.IDLE
            else:
                return lane_command, "LANE_COOLDOWN"

        # 여기 도달하면 state == IDLE 입니다.
        if obstacle_detected:
            self.state = self.AVOID_OUT
            self._state_until = now + cfg.AVOID_OUT_SECONDS
            print(
                "AVOID_START: 내 차선(2차선) 전방 "
                f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm 이내 장애물 감지, "
                "1차선으로 회피합니다."
            )
            return self._steer_command(+1), "AVOID_OUT"

        return lane_command, "LANE"
