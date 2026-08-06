"""모듈형 자율주행 프로그램의 메인 실행 파일입니다.

[버전: drive_core] 차선 추종만 남긴 최소 구성. 다른 버전들(drive, v2~v5)이
동시에 여러 기능을 실차 검증 없이 쌓다가 얽혀서, 여기서부터 차선 추종을
완전히 안정시킨 뒤 신호등 → 정지선 순으로 하나씩 실차 검증하며 추가할
예정입니다. resolve_drive_command는 지금은 차선 결과를 그대로 통과시키지만,
나중에 기능을 얹을 자리로 그대로 남겨뒀습니다.

라이다 장애물 회피(obstacle_detector.py)는 차선 추종과 별개 레이어로
추가했습니다. [2026-08-06 Claude 수정 3] 이제 조향은 항상 카메라 차선
추종(lane_controller)이 계산합니다 — AvoidanceController는 매 프레임
카메라 처리 "전"에 먼저 호출해 필요하면 lane_controller에 차선 오프셋을
지시하고(begin_frame), 카메라 처리 "후"에 다시 호출해 최종 속도/조향과
로그용 reason을 받습니다(finalize_frame). 자세한 임계값/기동 파라미터는
config.py의 "라이다 장애물 감지" / "장애물 회피" 섹션을 보세요.
"""

import time

import cv2

import config as cfg
from hardware_controller import HardwareController
from lane_controller import LaneController
from obstacle_detector import AvoidanceController, LidarObstacleDetector


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


def draw_status(frame, lane_result, command, fps, obstacle_detected, lidar_debug):
    lidar_color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
    cv2.putText(
        frame,
        f"LIDAR:{'DETECT' if obstacle_detected else 'CLEAR'}",
        (8, 138),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        lidar_color,
        2,
    )
    if lidar_debug is not None:
        # LIDAR_FRONT_ANGLE/LIDAR_OBSTACLE_LATERAL_*_MM 실차 검증용: 가장
        # 가까운 점의 횡방향(LAT)/전방(FWD) 거리와 장애물 예상 구간
        # 안(IN)/밖(OUT)입니다. LAT 절대값이 50mm 경계를 지날 때 IN이
        # 뒤집히는지 확인하세요.
        in_lane_color = (0, 255, 0) if lidar_debug["in_lane"] else (0, 165, 255)
        cv2.putText(
            frame,
            (
                f"LAT:{lidar_debug['lateral']:.0f} "
                f"FWD:{lidar_debug['forward']:.0f} "
                f"IN:{'Y' if lidar_debug['in_lane'] else 'N'}"
            ),
            (160, 138),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            in_lane_color,
            2,
        )
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
            f"OFS:{lane_result.get('lane_offset_lanes', 0.0):.1f} "
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
    lidar_detector = LidarObstacleDetector()
    avoidance = AvoidanceController()

    if not wait_for_start(hardware):
        hardware.stop()
        hardware.close()
        lidar_detector.stop()
        return

    frame_count = 0
    frame_lost_count = 0
    start_time = time.monotonic()
    # 회피 기동 중 카메라 프레임이 잠깐 끊겨도 조향이 차선 추종값으로
    # 되돌아가지 않도록, 실제로 마지막에 내보낸 명령을 따로 기억합니다.
    last_command = {"speed": 0, "steering": cfg.STEER_CENTER}

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
                        last_command["speed"],
                        last_command["steering"],
                    )
                continue

            frame_lost_count = 0
            frame_count += 1

            # 라이다는 카메라와 무관하므로 먼저 확인하고, lane_controller.
            # update(frame) 전에 차선 오프셋을 지시해야 이번 프레임의
            # 카메라 제어 계산에 바로 반영됩니다.
            obstacle_detected = lidar_detector.is_obstacle_detected()
            lidar_debug = lidar_detector.get_debug_info()
            avoidance.begin_frame(obstacle_detected, lane_controller)

            lane_result = lane_controller.update(frame)
            lane_command = resolve_drive_command(lane_result)
            command, reason = avoidance.finalize_frame(lane_command)
            command["reason"] = reason
            command["sent_speed"] = hardware.drive(
                command["speed"], command["steering"]
            )
            last_command = {
                "speed": command["speed"],
                "steering": command["steering"],
            }

            fps = frame_count / max(
                time.monotonic() - start_time,
                0.001,
            )
            draw_status(frame, lane_result, command, fps, obstacle_detected, lidar_debug)
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
                    f"offset={lane_result.get('lane_offset_lanes', 0.0):.1f} "
                    f"lidar={'DETECT' if obstacle_detected else 'clear'} "
                    + (
                        f"lat={lidar_debug['lateral']:.0f} "
                        f"fwd={lidar_debug['forward']:.0f} "
                        f"in_lane={lidar_debug['in_lane']} "
                        if lidar_debug is not None
                        else "lat=- fwd=- in_lane=- "
                    )
                    + f"avoid_state={avoidance.state} "
                    f"reason={command['reason']} "
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
        lidar_detector.stop()


if __name__ == "__main__":
    main()
