"""라이다가 "내 차선(2차선)" 코리더 안의 물체만 잡는지, 모터/카메라 없이
라이다만 켜서 콘솔로 확인하는 도구입니다. main.py 대신 이 파일만 단독으로
실행합니다.

전제 조건:
  config.py의 LIDAR_FRONT_ANGLE이 calibrate_lidar_angle.py로 이미 실차에서
  캘리브레이션되어 있어야 합니다. 아직 임시값(0.0)이면 정면 기준 자체가
  틀어져 있어서 이 스크립트의 결과가 무의미합니다 — 이 파일보다
  calibrate_lidar_angle.py를 먼저 실행하세요.

모터/카메라는 전혀 사용하지 않고 라이다만 켜므로, 차량 바퀴가 지면에 닿은
상태에서 실행해도 안전합니다. main.py와 완전히 동일한 obstacle_detector.
LidarObstacleDetector를 그대로 재사용하므로(판정 기준이 서로 다른 사본을
만들지 않기 위함), 여기서 확인한 결과가 곧 실제 주행 중 판정과 같습니다.

사용법:
  1. 차량을 트랙(또는 아무 바닥) 위에 놓고 이 스크립트를 실행합니다.
  2. 물체(손, 상자 등)를 차량 정면의 여러 위치에 놓아 봅니다.
       - 장애물 예상 구간 안(트랙 40~64cm, lateral -110~+130mm) -> IN이어야 함
       - 그 밖(예: 1차선이나 트랙 바깥) -> OUT이어야 함
     동시에 여러 거리(가까이/멀리)에서도 테스트해보세요 — 코리더 방식은
     거리가 달라져도 lateral 판정 기준이 바뀌지 않아야 정상입니다.
  3. 콘솔에 매 스캔 "DETECT/clear"(연속 확정 여부), 감지범위 안 가장
     가까운 점의 lateral(횡방향)/forward(전방)/distance(직선거리), 그리고
     "IN"(장애물 예상 구간 안)/"OUT"(그 밖)이 계속 출력됩니다. lateral
     lateral이 40cm/64cm 경계(약 -110/+130mm)를 지날 때 IN/OUT이 정확히
     뒤집히는지 확인하세요.
  4. Ctrl+C로 종료합니다(라이다 모터가 자동으로 정지합니다).
"""

import time

import config as cfg
from obstacle_detector import LidarObstacleDetector

# 콘솔 출력 주기(초). 라이다 스캔 자체는 이 값과 무관하게 백그라운드
# 스레드에서 계속 돕니다 — 이건 사람이 읽기 편한 속도로 화면만 갱신합니다.
PRINT_INTERVAL_SECONDS = 0.1


def main():
    print(
        f"LIDAR_FRONT_ANGLE={cfg.LIDAR_FRONT_ANGLE:.1f}도, "
        f"장애물 예상 구간(LIDAR_OBSTACLE_LATERAL_MIN/MAX_MM)="
        f"{cfg.LIDAR_OBSTACLE_LATERAL_MIN_MM:.0f}~"
        f"{cfg.LIDAR_OBSTACLE_LATERAL_MAX_MM:.0f}mm, "
        f"감지거리={cfg.LIDAR_DETECT_MIN_DISTANCE_MM:.0f}~"
        f"{cfg.LIDAR_DETECT_MAX_DISTANCE_MM:.0f}mm"
    )
    if cfg.LIDAR_FRONT_ANGLE == 0.0:
        print(
            "경고: LIDAR_FRONT_ANGLE이 아직 임시값(0.0)입니다. "
            "calibrate_lidar_angle.py를 먼저 실행해 실제 정면 각도를 "
            "구해 config.py에 반영한 뒤 이 스크립트를 실행하세요."
        )
    print("모터/카메라는 켜지 않습니다. Ctrl+C로 종료합니다.\n")

    detector = LidarObstacleDetector()
    try:
        while True:
            detected = detector.is_obstacle_detected()
            debug = detector.get_debug_info()
            status = "DETECT" if detected else "clear "
            if debug is None:
                print(f"{status} | 감지거리 범위 안에 점 없음")
            else:
                in_lane = "IN " if debug["in_lane"] else "OUT"
                print(
                    f"{status} | "
                    f"lateral={debug['lateral']:7.1f}mm "
                    f"forward={debug['forward']:7.1f}mm "
                    f"distance={debug['distance']:7.1f}mm "
                    f"{in_lane}"
                )
            time.sleep(PRINT_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        print("\n라이다를 정지했습니다.")


if __name__ == "__main__":
    main()
