"""모듈형 자율주행 프로그램의 메인 실행 파일입니다.

[버전: drive_core] 차선 추종만 남긴 최소 구성. 다른 버전들(drive, v2~v5)이
동시에 여러 기능을 실차 검증 없이 쌓다가 얽혀서, 여기서부터 차선 추종을
완전히 안정시킨 뒤 신호등 → 정지선 순으로 하나씩 실차 검증하며 추가할
예정입니다. resolve_drive_command는 지금은 차선 결과를 그대로 통과시키지만,
나중에 기능을 얹을 자리로 그대로 남겨뒀습니다.
"""

import time

import cv2

import config as cfg
from hardware_controller import HardwareController
from lane_controller import LaneController


def wait_for_start(hardware):
    """사용자가 출발키를 누를 때까지 차량을 대기시킵니다."""
    print("대기 중: 출발은 's', 종료는 'q' 또는 ESC를 누르세요.")
    while True:
        frame = hardware.read_frame()
        if frame is not None:
            cv2.putText(
                frame,
                "Press 's' to START",
                (35, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
            )
            cv2.imshow(cfg.WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            return True
        if key in (ord("q"), 27):
            return False


def resolve_drive_command(lane_result):
    """차선 결과를 최종 속도/조향으로 변환합니다.

    지금은 차선 추종만 있어서 그대로 통과시킵니다. 신호등/정지선을 다시
    추가할 때 여기에 우선순위 분기를 넣습니다.
    """
    return {
        "speed": lane_result["speed"],
        "steering": lane_result["steering"],
        "reason": "LANE" if lane_result["speed"] > 0 else f"LANE_{lane_result['status']}",
    }


def draw_status(frame, lane_result, command, fps):
    cv2.putText(
        frame,
        (
            f"LANE:{lane_result['status']} "
            f"N:{lane_result['near_source']} "
            f"F:{lane_result['far_source']} "
            f"ERR:{lane_result['center_error']:.0f} "
            f"DIR:{lane_result['heading_error']:.0f} "
            f"CONF:{lane_result.get('confidence', 0.0):.2f}"
        ),
        (8, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        (
            f"SPD:{command['speed']} "
            f"OUT:{command.get('sent_speed', command['speed'])} "
            f"STR:{command['steering']} "
            f"MODE:{command['reason']} FPS:{fps:.1f}"
        ),
        (8, 86),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        (
            f"PERR:{lane_result.get('preview_error', 0.0):.0f} "
            f"PW:{lane_result.get('preview_weight', 0.0):.2f} "
            f"RG:{lane_result.get('guard', 0.0):.0f} "
            f"LG:{lane_result.get('left_guard', 0.0):.0f} "
            f"H:{lane_result.get('hard_boundary', 0)}"
        ),
        (8, 112),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 0),
        2,
    )
    if cfg.SHOW_DEBUG:
        LaneController.draw_debug(frame, lane_result)
    return frame


def main():
    hardware = HardwareController()
    lane_controller = LaneController()

    if not wait_for_start(hardware):
        hardware.stop()
        hardware.close()
        return

    frame_count = 0
    frame_lost_count = 0
    start_time = time.monotonic()

    try:
        while True:
            frame = hardware.read_frame()
            if frame is None:
                frame_lost_count += 1
                if frame_lost_count == cfg.FRAME_STOP_FRAMES:
                    # 실제 서보가 중앙으로 복귀한 상태와 내부 제어 기억을
                    # 일치시키고, 영상 복구 뒤 차선을 다시 확인합니다.
                    lane_controller.reset_tracking()
                    print(
                        "STOP_REASON=CAMERA_FRAME_LOST "
                        f"misses={frame_lost_count}"
                    )
                if frame_lost_count >= cfg.FRAME_STOP_FRAMES:
                    hardware.stop()
                else:
                    hardware.drive(
                        lane_controller.previous_speed,
                        lane_controller.previous_steer,
                    )
                continue

            frame_lost_count = 0
            frame_count += 1
            lane_result = lane_controller.update(frame)

            command = resolve_drive_command(lane_result)
            command["sent_speed"] = hardware.drive(
                command["speed"], command["steering"]
            )

            fps = frame_count / max(
                time.monotonic() - start_time,
                0.001,
            )
            draw_status(frame, lane_result, command, fps)
            cv2.imshow(cfg.WINDOW_NAME, frame)
            if cfg.SHOW_DEBUG:
                cv2.imshow("Lane Mask", lane_result["mask"])

            if frame_count % cfg.PRINT_INTERVAL == 0:
                print(
                    f"lane={lane_result['status']} "
                    f"confidence={lane_result.get('confidence', 0.0):.2f} "
                    f"error={lane_result.get('center_error', 0.0):.1f} "
                    f"heading={lane_result.get('heading_error', 0.0):.1f} "
                    f"preview={lane_result.get('preview_error', 0.0):.1f} "
                    f"pweight={lane_result.get('preview_weight', 0.0):.2f} "
                    f"curvature={lane_result.get('curvature', 0.0):.1f} "
                    f"guard={lane_result.get('guard', 0.0):.1f} "
                    f"left_guard={lane_result.get('left_guard', 0.0):.1f} "
                    f"corridor={lane_result.get('corridor_steer', 0.0):.1f} "
                    f"hard={lane_result.get('hard_boundary', 0)} "
                    f"safety={lane_result.get('lateral_safety', 0.0):.1f} "
                    f"single={lane_result.get('single_lane_side')}:"
                    f"{lane_result.get('single_lane_frames', 0)} "
                    f"speed={command['speed']} "
                    f"output={command.get('sent_speed', command['speed'])} "
                    f"steer={command['steering']} "
                    f"fps={fps:.1f}"
                )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        hardware.stop()
        hardware.close()


if __name__ == "__main__":
    main()
