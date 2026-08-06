"""트랙 조명에 맞춰 차선 인식 색상 임계값을 현장에서 직접 눈으로 보며
맞추는 도구입니다. main.py 대신 이 파일만 단독으로 실행합니다.

아두이노는 연결하지 않고 카메라만 사용합니다(주행하지 않으므로 안전합니다).

사용법:
  1. 차를 실제 대회 트랙 위, 출발 지점에 놓고 이 스크립트를 실행합니다.
  2. V_MIN / S_MAX / ROI_TOP 트랙바를 조절하면서, 오른쪽 Mask 창에서
     실제 차선(점선/실선)만 하얗게 남고 바닥·반사광은 최대한 검게 나오도록
     맞춥니다. 차를 조금씩 밀어서 트랙의 여러 구간(직선, 커브)에서도
     확인해보는 것이 좋습니다.
  3. 만족스러우면 's'를 눌러 현재 값을 콘솔에 출력합니다.
  4. 그 값을 config.py의 LANE_WHITE_VALUE_MIN / LANE_WHITE_SATURATION_MAX /
     ROI_TOP에 그대로 옮겨 적습니다.
  5. 'q' 또는 ESC로 종료합니다.
"""

import cv2
import numpy as np

import config as cfg
import Function_Library as fl


def _nothing(_value):
    pass


def main():
    camera_api = fl.libCAMERA()
    camera, _ = camera_api.initial_setting(cam0port=cfg.CAMERA_PORT, capnum=1)
    if cfg.LOCK_CAMERA_EXPOSURE and camera is not None and hasattr(camera, "set"):
        camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        camera.set(cv2.CAP_PROP_EXPOSURE, cfg.CAMERA_EXPOSURE)
        camera.set(cv2.CAP_PROP_AUTO_WB, 0)
        camera.set(cv2.CAP_PROP_WB_TEMPERATURE, cfg.CAMERA_WB_TEMPERATURE)

    window = "Calibrate (yellow box = ROI)"
    cv2.namedWindow(window)
    cv2.createTrackbar(
        "V_MIN", window, cfg.LANE_WHITE_VALUE_MIN, 255, _nothing
    )
    cv2.createTrackbar(
        "S_MAX", window, cfg.LANE_WHITE_SATURATION_MAX, 255, _nothing
    )
    cv2.createTrackbar("ROI_TOP", window, cfg.ROI_TOP, 480, _nothing)

    print(
        "V_MIN / S_MAX / ROI_TOP 트랙바를 조절해서 Mask 창이 차선만 "
        "깨끗하게 잡도록 맞추세요."
    )
    print("직선과 커브 여러 구간에서 확인해보세요 (차를 손으로 밀어도 됩니다).")
    print("'s' = 현재 값 콘솔 출력, 'q' 또는 ESC = 종료")

    try:
        while True:
            _, frame = camera_api.camera_read(camera)
            if frame is None:
                continue

            v_min = cv2.getTrackbarPos("V_MIN", window)
            s_max = cv2.getTrackbarPos("S_MAX", window)
            roi_top = cv2.getTrackbarPos("ROI_TOP", window)

            bottom = min(cfg.ROI_BOTTOM, frame.shape[0])
            top = min(roi_top, bottom - 1)
            roi = frame[top:bottom, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            saturation = hsv[:, :, 1]
            value = hsv[:, :, 2]

            white = (
                (value >= v_min) & (saturation <= s_max)
            ).astype(np.uint8) * 255
            mask = cv2.morphologyEx(
                white,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)),
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            )
            white_ratio = 100.0 * float((mask > 0).mean())

            display_frame = frame.copy()
            cv2.rectangle(
                display_frame,
                (0, top),
                (frame.shape[1] - 1, bottom - 1),
                (0, 255, 255),
                2,
            )
            cv2.putText(
                display_frame,
                f"white={white_ratio:.1f}%  (line-only should be roughly 2~6%)",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            cv2.imshow(window, display_frame)
            cv2.imshow("Mask", mask_bgr)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                print(
                    "----- 현재 값 (config.py에 붙여넣으세요) -----\n"
                    f"LANE_WHITE_VALUE_MIN = {v_min}\n"
                    f"LANE_WHITE_SATURATION_MAX = {s_max}\n"
                    f"ROI_TOP = {roi_top}\n"
                    f"(지금 이 순간 흰색 비율: {white_ratio:.1f}%)\n"
                    "-----------------------------------------"
                )
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
