"""라이다 장착 방향에 따라 "차량 정면"이 스캔 각도 몇 도에 해당하는지
현장에서 직접 확인하는 도구입니다. main.py 대신 이 파일만 단독으로
실행합니다.

모터/카메라는 사용하지 않고 라이다만 켜므로, 차량 바퀴가 지면에 닿은
상태에서 실행해도 안전합니다.

사용법:
  1. 차량을 실제 대회 트랙(또는 아무 바닥) 위에 놓고 이 스크립트를 실행합니다.
  2. 차량 정면 30~50cm 앞에 손이나 상자를 놓습니다.
  3. 콘솔에 계속 출력되는 "가장 가까운 점"의 각도를 확인합니다. 물체를
     정면에 정확히 놓았을 때 나오는 각도가 곧 정면 각도입니다.
  4. 그 값을 config.py의 LIDAR_FRONT_ANGLE에 그대로 옮겨 적습니다.
  5. Ctrl+C로 종료합니다(라이다 모터가 자동으로 정지합니다).
"""

import Function_Library as fl
import config as cfg

# 몇 mm 이내의 점만 "내가 놓은 물체"로 볼지 결정합니다. 트랙 벽 등 먼
# 물체가 같이 잡히지 않도록 config의 감지 거리보다 약간 좁게 잡았습니다.
CALIBRATION_MAX_DISTANCE_MM = 600


def main():
    lidar = fl.libLIDAR(cfg.LIDAR_PORT)
    lidar.init()
    print(f"라이다 포트 {cfg.LIDAR_PORT} 연결 완료.")
    print("차량 정면 30~50cm 앞에 손이나 상자를 놓아주세요.")
    print("Ctrl+C로 종료합니다.\n")

    try:
        for scan in lidar.scanning():
            if len(scan) == 0:
                continue

            nearby = lidar.getDistanceRange(
                scan, 0, CALIBRATION_MAX_DISTANCE_MM
            )
            if len(nearby) == 0:
                print(
                    f"{CALIBRATION_MAX_DISTANCE_MM}mm 이내에 아무것도 "
                    "없습니다... 물체를 좀 더 가까이 놓아보세요."
                )
                continue

            closest = nearby[nearby[:, 1].argmin()]
            angle, distance = float(closest[0]), float(closest[1])
            print(f"각도={angle:6.1f}도   거리={distance:6.0f}mm")
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        print("\n라이다를 정지했습니다.")


if __name__ == "__main__":
    main()
