"""자율주행 차량에서 조정할 수 있는 모든 설정값.

[drive_core] 차선 추종만 남긴 최소 구성입니다. 신호등/정지선/직진모드/주차는
아직 없습니다 — 차선 추종이 실차에서 안정되면 하나씩 실차 검증하며 추가할
예정입니다. drive_v2에서 오늘 실차로 검증된 값 그대로 가져왔습니다.

[2026-08-06 01:17 Claude 수정] 박스형 표시(주차 테스트 칸) 오검출 필터
BOX_FILTER_* 상수군을 새로 추가했습니다(아래 "박스형 표시 오검출 필터"
블록). lane_controller.py의 _remove_box_structures / _confirm_horizontal_bars
/ _group_bars_into_ladders 와 짝입니다. RIGHT_GUARD_GAIN 등 기존 값은
건드리지 않았습니다.

[2026-08-06 Claude 수정] config_by_claude.py에 따로 있던 라이다 장애물
회피 설정(LIDAR_*, AVOID_*)을 이 파일 맨 끝으로 병합했습니다. 파일이
갈라져 있으면 어느 쪽을 고쳐도 실행 중인 main.py에는 반영이 안 되는
문제가 있어서 config.py 하나로 합쳤습니다. 병합하며 기존 ARDUINO_PORT/
CAMERA_PORT(COM9/1) 등 이 파일의 값은 그대로 두었으니, 다른 PC에서
테스트했다면(COM12/2였음) 실제 장치에 맞게 다시 확인하세요.

[2026-08-06 Claude 수정 2] "2차선(바깥 차선)만 달리고, 옆 1차선의
장애물은 무시하며, 피한 뒤엔 다시 2차선으로 복귀해야 한다"는 요구사항
반영. LIDAR_DETECT_HALF_WINDOW_DEG(각도창) 대신 차량 진행축 기준 좌우
폭 코리더(LidarObstacleDetector._hit_in_lane_corridor)로 판정 방식을
바꾸고, AVOID_DURATION_SECONDS 하나였던 회피 기동을 AVOID_OUT/HOLD/
RETURN_SECONDS 세 구간으로 나눠 "피하고 다시 돌아오기"를 지원합니다.
자세한 설계 이유는 아래 "라이다 장애물 감지" / "장애물 회피" 섹션
주석과 obstacle_detector.py를 보세요. AVOID_HOLD_SECONDS는 아직 실차
튜닝 전 추정값이라 실제 장애물 폭/차 길이에 맞게 반드시 재검증해야
합니다.

[2026-08-06 Claude 수정 3] 장애물 감지 판정 좌표를 트랙 전체 폭 기준
(0/34/68cm 세 선)으로 다시 정리하고, 감지 구간을 실측된 장애물 위치
(트랙 전체 기준 46~56cm, 우리 차선 중앙 51cm에서 ±5cm)에 맞춰
LIDAR_OBSTACLE_LATERAL_MAX_MM을 230 -> 50으로 좁혔습니다. LIDAR_LANE_
WIDTH_MM도 우리 차선 하나의 실제 폭(340mm)으로 바로잡았습니다(예전엔
트랙 전체 폭 680mm를 우리 차선 폭으로 잘못 썼습니다). 자세한 내용은
아래 "라이다 장애물 감지" 섹션 주석을 보세요.

[2026-08-06 Claude 수정 4] 회피 기동 방식을 오픈루프(OUT/HOLD/RETURN
3단계, 정해진 각도로 정해진 시간만 꺾기)에서 카메라 기반 차선 오프셋
방식으로 완전히 바꿨습니다. 실차에서 각도/시간 조합이 조금만 어긋나도
과회전(카메라가 트랙 밖을 봄)하거나 못 미치는 문제가 반복돼서, 이제는
AvoidanceController가 LaneController.set_lane_offset()으로 "목표
차선을 옆으로 옮겨라"라고만 지시하고, 실제 조향은 매 프레임 카메라
차선 추종 PID가 계속 담당합니다. AVOID_STEER_OFFSET/AVOID_OUT_SECONDS/
AVOID_RETURN_SECONDS는 삭제했고, AVOID_HOLD_SECONDS는 AVOID_DURATION_
SECONDS로 이름을 바꿔 "오프셋을 얼마나 유지할지"라는 뜻으로만 씁니다.
자세한 내용은 "장애물 회피" 섹션과 lane_controller.py의
set_lane_offset/_calculate_control, obstacle_detector.py를 보세요.

[2026-08-06 Claude 수정 5] "4초 로직을 버리고 장애물이 있을 때만
차선을 변경하라"는 요구사항 반영. AVOID_DURATION_SECONDS/AVOID_
COOLDOWN_SECONDS를 완전히 삭제했습니다.

[2026-08-06 Claude 수정 6->7 (최종)] 처음엔 "장애물 미감지시 자동 복귀"
(수정 5), 그 다음엔 "점선을 넘은 게 카메라로 확인되면 자동 복귀"(수정 6)
로 시도했지만, 둘 다 "자동 복귀" 자체가 요구사항이 아니라는 걸 알게
됐습니다. 최종적으로 자동 복귀 개념을 완전히 없애고, 장애물을 새로
감지할 때마다 지금 있는 차선의 옆 차선으로 토글하는 방식으로
정리했습니다(obstacle_detector.AvoidanceController 참고). 2차선에서
감지되면 1차선으로, 그 뒤 1차선에서 또 감지되면 다시 2차선으로 —
감지 이벤트가 없으면 마지막 설정을 계속 유지합니다.
"""

# 하드웨어 연결 설정
ARDUINO_PORT = "COM9"
BAUD_RATE = 9600
CAMERA_PORT = 1
STEER_CENTER = 157
# 시리얼 쓰기가 실패했을 때(케이블 순간 접촉 불량 등) 재연결을 시도하는
# 최소 간격(초). 너무 짧으면 매 프레임 재연결을 시도해 루프가 계속
# 지연됩니다(재연결 자체가 아두이노 리셋 대기로 약 2초 걸립니다).
SERIAL_RECONNECT_INTERVAL = 2.0

# 카메라 설정
LOCK_CAMERA_EXPOSURE = True
CAMERA_EXPOSURE = -6
CAMERA_WB_TEMPERATURE = 4500
FRAME_STOP_FRAMES = 3

# 차선 영상 처리 설정
ROI_TOP = 200
ROI_BOTTOM = 480
# 흰색 차선은 밝고 채도가 낮아야 합니다. 밝기만 사용하면 초록 바닥도
# 흰색으로 처리되므로 HSV 채도 조건을 반드시 함께 사용합니다.
WHITE_THRESHOLD = 200
LANE_WHITE_VALUE_MIN = 170
LANE_WHITE_SATURATION_MAX = 65
# 회색 바닥(주변과 밝기 차 작음)만 걸러내고, 희미한 점선은 남깁니다.
LANE_LOCAL_CONTRAST_MIN = 10.0
LANE_LOCAL_BLUR_SIZE = 31
# 넓은 회색 덩어리 제거. 얇은 세로 선(점선/실선)은 aspect로 보호합니다.
MASK_FLOOR_BLOB_MAX_AREA = 2800.0
MASK_FLOOR_BLOB_MAX_ASPECT = 5.0
LANE_MASK_PARKING_MAX_X = 130.0
LANE_MASK_MIN_LINE_ASPECT = 3.5
LANE_MASK_MIN_DASH_AREA = 40.0
REFERENCE_WIDTH = 640.0

# [2026-08-05] 박스형 표시(주차 테스트 칸 등) 오검출 필터. 실차 영상
# (20260805-1032-45, 44초 부근)에서 박스 내부의 세로 분할선을 차선으로
# 착각해 조향이 끌려가는 걸 확인했다. LaneController._remove_box_
# structures 참고(연결요소 방식은 박스 변들이 서로 맞닿아 실패해서,
# 가로 방향 모폴로지 열기로 바꿨다 - 함수 docstring 참고).
# 값 튜닝 가이드:
#   HORIZONTAL_KERNEL_WIDTH는 TRACK_MAX_SEGMENT_WIDTH(45px, 640기준)보다
#   충분히 커야 진짜 차선을 안 지우고, 박스 가로변 길이(실측 필요)보다는
#   작아야 박스를 잡는다. 지금 80px은 그 사이 안전값이다.
#   (커널 높이를 1보다 키우거나 가로 방향 닫기로 끊긴 변을 잇는 시도는
#   해봤지만, 실측에서 진짜 오른쪽 실선이 통째로 지워지는 심각한
#   부작용이 나와서 되돌렸다 - LaneController._remove_box_structures
#   docstring 참고. 커널 높이는 항상 1로 고정.)
#   X/Y_MARGIN을 넓히면 박스 주변에서 더 넓게 지운다(과하면 박스 옆의
#   진짜 차선까지 지울 위험 - 실차에서 점선이 남아있는지 꼭 확인할 것).
#
# [2026-08-05 3차] 가로 연속 길이만으로 판정하면 급커브 정점의 차선
# (그 순간 실제로 가로로 눕는다)까지 지워버리는 걸 전체 랩 재생으로
# 확인했다. 그래서 아래 세 조건을 모두 만족하는 후보만 박스 변으로
# 인정한다(_confirm_horizontal_bars). 실측 대조값:
#                    박스 가로변(t=44)   급커브 차선(t=11)
#   MAX_BAR_VY       0.008 / 0.015      0.988   (방향이 수평인가)
#   MIN_BAR_ASPECT   17.6 / 11.8        0.67    (얇고 긴가)
#   MAX_BAR_RESIDUAL 2.6 / 3.0          13.7    (곧은가)
# 값을 조일수록(VY/RESIDUAL 낮추고 ASPECT 올림) 진짜 차선을 지울
# 위험은 줄지만 박스를 놓칠 수 있다. 실차에서 박스가 그대로 남으면
# 완화하고, 커브에서 차선이 사라지면 조인다.
BOX_FILTER_ENABLED = True
BOX_FILTER_HORIZONTAL_KERNEL_WIDTH = 80.0
BOX_FILTER_MIN_BAR_AREA = 100
BOX_FILTER_MIN_BAR_ASPECT = 4.0
BOX_FILTER_MAX_BAR_VY = 0.25
BOX_FILTER_MAX_BAR_RESIDUAL = 6.0
# [2026-08-05 4차] 막대 하나만으로는 박스와 배경(밝은 체육관 나무 바닥의
# 이음새 선도 얇고 곧고 수평이다)이 구분되지 않아, 그 선에 연결된 거대한
# 배경 덩어리가 통째로 지워지는 문제가 있었다(프레임당 최대 3만 픽셀,
# 33.97~46.73초 내내). 박스의 진짜 특징은 가로 변이 최소 2개 나란히
# 있는 "사다리" 구조이므로, 아래 조건으로 짝을 이루는 막대만 박스로
# 인정한다(_group_bars_into_ladders). 실측 박스 두 변 간격은 78px
# (640기준)이었다.
BOX_FILTER_LADDER_MIN_GAP = 25.0
BOX_FILTER_LADDER_MAX_GAP = 200.0
BOX_FILTER_LADDER_MIN_OVERLAP = 0.30
BOX_FILTER_X_MARGIN = 8.0
BOX_FILTER_Y_MARGIN = 40.0
# [2026-08-06 시도했다가 되돌림] 차가 박스에 바짝 붙으면 가로 테두리가
# ROI 밖으로 잘려나가 사다리 판정 자체가 안 되는 근접 구간 대응으로,
# 한 번 찾은 사다리 위치를 잠시 기억해 계속 지우는 BOX_FILTER_MEMORY_
# FRAMES 기능을 넣었다가 뺐다. 전체 랩 재생 검증(LaneController.
# _remove_box_structures docstring 참고)에서 LOST가 122->308로 급증하는
# 심각한 부작용이 나와 상수까지 완전히 제거했다. 근접 구간 세로
# 분할선 잔존은 아직 미해결.

# 이전 주행 코드에서 측정한 가까운 구간과 먼 구간의 차선 위치
LANE_BANDS = (
    {
        "name": "NEAR",
        "y": (0.75, 0.95),
        "right_range": (250, 639),
        "left": 100.0,
        "right": 540.0,
        "width": 440.0,
        "width_range": (300, 560),
        "target": 320.0,
    },
    {
        "name": "FAR",
        "y": (0.05, 0.20),
        "right_range": (250, 620),
        "left": 220.0,
        "right": 400.0,
        "width": 180.0,
        "width_range": (100, 280),
    },
)

# 다중 높이 차선 추적 설정
# 오른쪽 실선을 먼저 연속 추적한 뒤, 보이는 중앙 점선으로 원근 차선 폭을
# 학습합니다. 점선이 끊긴 곳은 학습한 폭으로 중앙을 복원합니다.
TRACK_ROW_COUNT = 15
TRACK_TOP_RATIO = 0.05
TRACK_BOTTOM_RATIO = 0.96
TRACK_STRIP_HALF_HEIGHT = 5
TRACK_COLUMN_MIN_RATIO = 0.36
TRACK_MIN_SEGMENT_WIDTH = 2
TRACK_MAX_SEGMENT_WIDTH = 45
TRACK_RIGHT_MIN_X = 240.0
TRACK_RIGHT_MIN_OFFSET_RATIO = 0.20
TRACK_RIGHT_LOCKED_MIN_OFFSET_RATIO = -0.05
TRACK_RIGHT_LOCKED_SEARCH_WIDTH_RATIO = 0.55
TRACK_RIGHT_SEARCH_MARGIN = 70.0
TRACK_RIGHT_GAP_GROWTH = 45.0
TRACK_SEARCH_MARGIN = 70.0
TRACK_MIN_POINTS = 6
TRACK_POLY_DEGREE = 3
TRACK_RESIDUAL_LIMIT = 18.0
TRACK_MAX_FIT_RMSE = 22.0
TRACK_MIN_COVERAGE = 0.42
TRACK_MIN_CONFIDENCE = 0.55
TRACK_START_CONFIRM_FRAMES = 3
TRACK_PATH_FILTER = 0.65
TRACK_MAX_NEAR_JUMP = 35.0
TRACK_MAX_MIDDLE_JUMP = 50.0
TRACK_MAX_PREVIEW_JUMP = 65.0
TRACK_MAX_FAR_JUMP = 80.0
TRACK_CUBIC_MIN_BOTH_POINTS = 6
TRACK_CUBIC_MIN_BOTH_COVERAGE = 0.38
TRACK_CUBIC_RMSE_RATIO = 0.76
TRACK_MIN_LANE_WIDTH = 25.0
TRACK_MAX_LANE_WIDTH = 625.0
TRACK_WIDTH_MODEL_MIN_SLOPE = 0.08
TRACK_WIDTH_MODEL_MAX_SLOPE = 1.30
TRACK_WIDTH_PAIR_TOLERANCE = 22.0
TRACK_WIDTH_PAIR_TOLERANCE_RATIO = 0.11
TRACK_WIDTH_SEED_TOLERANCE = 150.0
TRACK_WIDTH_SECOND_TOLERANCE = 45.0
TRACK_WIDTH_MIN_PAIR_ROWS = 3
TRACK_WIDTH_MIN_PAIR_COVERAGE = 0.16
TRACK_WIDTH_FILTER = 0.82
TRACK_PAIR_CENTER_BLEND = 0.45

# -------------------------------------------------------------------
# 중앙선(1·2차선 사이 점선) 전용 추종 모드
# -------------------------------------------------------------------
# True면 좌우 경계/차선 폭 추정 대신 점선만 직접 추적합니다. 회색 바닥을
# 왼쪽 차선으로 오인하는 문제를 피하고, 차선 변경 시 점선이 화면 왼쪽↔
# 오른쪽으로 이동하는 것을 정상 주행으로 취급합니다.
CENTER_LINE_MODE = False
# 2차선(기본)에서 점선이 보이길 원하는 화면 X(640기준). 회피 오프셋은
# 여기에 차선 폭을 더해 1차선(점선이 오른쪽) 목표를 만듭니다.
CENTER_LINE_TARGET_X = 220.0
CENTER_LINE_MIN_POINTS = 5
CENTER_LINE_MIN_COVERAGE = 0.28
CENTER_LINE_MIN_CONFIDENCE = 0.30
CENTER_LINE_MAX_SEGMENT_WIDTH = 28.0
CENTER_LINE_SEARCH_MARGIN = 90.0
CENTER_LINE_GAP_GROWTH = 55.0
CENTER_LINE_SEARCH_MIN_RATIO = 0.08
CENTER_LINE_SEARCH_MAX_RATIO = 0.72
# 차선 변경 시 점선이 화면 한쪽→다른 쪽으로 크게 움직여도 허용합니다.
CENTER_LINE_MAX_NEAR_JUMP = 200.0
CENTER_LINE_MAX_MIDDLE_JUMP = 240.0
CENTER_LINE_MAX_PREVIEW_JUMP = 280.0
CENTER_LINE_MAX_FAR_JUMP = 320.0
CENTER_LINE_MAX_FIT_RMSE = 28.0

# 왼쪽 경계(점선) 후보로 쓸 세그먼트 최대 폭(px, 640기준). 이보다 넓은
# 흰 덩어리는 회색 바닥/반사로 보고 제외합니다(점선 조각은 얇음).
LEFT_BOUNDARY_MAX_SEGMENT_WIDTH = 30.0
# 회피 중(차선 오프셋 활성)에는 왼쪽 경계를 쓰지 않고 오른쪽 실선+학습
# 폭만으로 중앙을 복원합니다. 왼쪽으로 이동할 때 바닥이 차선으로
# 잡혀 BOTH 판정이 깨지는 문제를 막습니다.
AVOID_RIGHT_ONLY_TRACKING = True
# 회피 중(OFS!=0)에만 왼쪽 ROI를 지울 때 쓰는 폭(px, 640기준).
LANE_MASK_CLEAR_LEFT_MAX_X_AVOID = 310.0
# 회피 중 차선이 잠깐 안 보여도 MEMORY를 더 오래 유지합니다(프레임).
AVOID_MEMORY_FRAMES = 90

# 회피 중에는 오른쪽 실선+학습 폭만 씁니다(왼쪽 바닥 오인식 방지).
AVOID_RIGHT_ONLY_JUMP_NEAR = 120.0
AVOID_RIGHT_ONLY_JUMP_MIDDLE = 150.0
AVOID_RIGHT_ONLY_JUMP_PREVIEW = 180.0
AVOID_RIGHT_ONLY_JUMP_FAR = 220.0

# 한쪽 차선만 보일 때의 안전 추적 설정입니다. 양쪽 차선으로 폭을 먼저
# 학습한 뒤에만 사용하며, 보이는 경계와 저장된 폭으로 중앙을 복원합니다.
SINGLE_LANE_ENABLED = True
SINGLE_LANE_MIN_POINTS = 5
SINGLE_LANE_MIN_COVERAGE = 0.28
SINGLE_LANE_MAX_RMSE = 16.0
SINGLE_LANE_RESIDUAL_LIMIT = 18.0
SINGLE_LANE_SEARCH_MARGIN = 75.0
SINGLE_LANE_SIDE_OFFSET_RATIO = 0.20
SINGLE_LANE_MAX_NEAR_JUMP = 55.0
SINGLE_LANE_MAX_MIDDLE_JUMP = 80.0
SINGLE_LANE_MAX_FAR_JUMP = 120.0
SINGLE_LANE_MAX_FRAMES = 300
SINGLE_LANE_WIDTH_MAX_AGE = 300
SINGLE_LANE_SPEED = 105

# 경로 신뢰성 검사(2026-08-04 실차 테스트에서 추가). 커튼 주름이나 주차칸
# 격자선처럼 차선이 아닌 흰색 물체를 SINGLE_LANE이 붙잡으면, 프레임별
# 이동 제한(TRACK_MAX_*_JUMP)은 통과하면서도 여러 프레임에 걸쳐 서서히
# heading/curvature가 실제 코스에서 나올 수 없는 값까지 치솟았습니다
# (신호등 게이트 기둥 근처에서 두 번, heading이 -417까지 튀며 차가 기둥에
# 붙어 사람이 손으로 잡아야 했음). 이 값을 넘는 경로는 이번 프레임을
# 버리고 직전 경로를 유지합니다(MEMORY로 폴백).
#
# 처음엔 curvature 120 / heading 200으로 잡았다가 두 가지 문제를 실차에서
# 확인해서 넓혔습니다:
# 1) S자 구간은 confidence 0.6~0.75의 정상 TRACKING 상태에서도 curvature가
#    92~239, heading이 -210 근방까지 legitimate하게 올라갑니다 — 120/200은
#    이 정상 구간까지 오검출로 취급해서 차가 S자에서 자꾸 멈칫거리게
#    만들었습니다.
# 2) 출발 지점의 첫 경로가 heading -193~-199.9로 임계값(200) 바로 아래를
#    맴돌면서, 첫 락온(previous_coefficients=None)마저 계속 거부되어 차가
#    30초 넘게 전혀 출발을 못 한 사례가 있었습니다. 이제 첫 락온은 이
#    검사를 건너뛰도록 별도로 고쳤지만(_accept_and_filter_path 참고),
#    임계값 자체도 여유를 더 뒀습니다.
# 사고 당시 관측값(게이트: heading -417~-569, 주차구역: curvature 333/
# heading 975)은 여전히 넉넉히 걸러냅니다.
MAX_PLAUSIBLE_HEADING_ERROR = 320.0
MAX_PLAUSIBLE_CURVATURE = 280.0

# 잘못된 선 쌍으로 지나치게 좁은 차선 폭을 학습하지 않도록 검사합니다.
WIDTH_MODEL_NEAR_MIN = 280.0
WIDTH_MODEL_NEAR_MAX = 600.0
WIDTH_MODEL_FAR_MIN = 70.0
WIDTH_MODEL_FAR_MAX = 380.0
WIDTH_MODEL_MIN_GROWTH_RATIO = 1.25
WIDTH_MODEL_MAX_CHANGE_RATIO = 0.30
# (2026-08-04) 20프레임(30fps 기준 0.67초)은 벽 때문에 양쪽이 잠깐 다
# 안 보이는 순간을 버티기엔 짧았습니다. LOST 상태에서도 한쪽 선만으로
# 재락온을 시도하도록 고쳤지만(_try_single_lane), 애초에 LOST까지 안
# 가는 게 더 안전하므로 여유를 넉넉히 늘렸습니다.
MEMORY_FRAMES = 50
LOST_STOP_FRAMES = 4

# 경로 제어 위치: 가까운 곳, 중간, 먼 곳 순서입니다.
CONTROL_Y_RATIOS = (0.81, 0.45, 0.18)
# 가까운 경로만 카메라 전방축과 비교하고, 먼 경로는 고정 X를 쫓지 않고
# 진행 방향만 반영합니다. 사진에서 실제 가까운 차선 중앙은 약 320~335입니다.
CONTROL_TARGET_X = (320.0, 320.0, 320.0)
PREVIEW_Y_RATIO = 0.34
# [2026-08-06 Claude 수정] 0.10/0.08/0.18 -> 0.10/0.16/0.36.
# (처음엔 TRACKING_WEIGHT도 0.20으로 올렸다가 0.10으로 되돌림. 이 값은
#  속도와 무관한 "기본" 전방주시 비중이라, 올리면 저속에서까지 멀리
#  보게 된다. 발산은 속도가 붙을 때만 생기는 문제이므로 기본값은
#  건드리지 않고 속도 연동분(SPEED_GAIN)과 상한만 올리는 게 맞다.
#  결과: 저속에서는 수정 전과 완전히 동일, 최고속에서만 PW가
#  0.10+0.16=0.26으로 올라간다.)
# BOUNDARY_PREVIEW_MIN_SCALE과 같은 문제(전방주시가 너무 짧아 속도가
# 붙으면 발산)의 나머지 절반이다. min_scale만 고쳤을 때 hard 구간의
# PW=0 비율은 50.1%->0%로 없어졌지만, 상한이 0.18이라 실효 전방주시가
# forward 0.215->0.230(+7%)밖에 안 늘어 부족했다.
#   실효 전방주시 = (1-PW)*0.19 + PW*0.66
#   (근접 제어점 CONTROL_Y_RATIOS[0]=0.81 -> forward 0.19,
#    preview 제어점 PREVIEW_Y_RATIO=0.34 -> forward 0.66)
# 재생 검증(20260805-1032-45 한 바퀴): 상한을 0.55까지 올려도 LOST 122,
# TRACKING 561로 검출 결과는 전혀 안 바뀌고 오차 프레임간 변화만
# 5.01->5.14(+2.6%)였다. 검출 쪽 위험은 낮다.
# 판정: 진동 진폭이 줄면 성공. 커브에서 안쪽을 미리 파고들거나(코너
#   컷) 반응이 굼떠지면 과한 것이므로 0.14/0.10/0.24 정도로 낮출 것.
PREVIEW_TRACKING_WEIGHT = 0.10
PREVIEW_SPEED_GAIN = 0.16
PREVIEW_MAX_WEIGHT = 0.36
PREVIEW_SINGLE_WEIGHT = 0.15
PREVIEW_SINGLE_MIN_POINTS = 7
PREVIEW_SINGLE_MIN_COVERAGE = 0.45
PREVIEW_ERROR_LIMIT = 180.0
PREVIEW_SINGLE_RAMP_FRAMES = 3
HEADING_CONTROL_GAIN = 0.02
SIGNED_CURVATURE_GAIN = 0.01
CURVE_OUTSIDE_HEADING_GAIN = 0.05
CURVE_OUTSIDE_CURVATURE_GAIN = 0.03
CURVE_OUTSIDE_MAX = 14.0
CURVE_OUTSIDE_FILTER = 0.82
CURVE_OUTSIDE_SPEED_GAIN = 0.35
CONTROL_ERROR_LIMIT = 150.0
# 왼쪽으로 꺾이는 구간에서는 중앙 추정만 사용하지 않고 실제 왼쪽 점선에서
# 복원한 차로 중앙을 함께 사용합니다. 점선 공백은 짧게만 기억합니다.
LEFT_ANCHOR_CURVE_TRIGGER = 8.0
LEFT_ANCHOR_BLEND = 0.75
LEFT_ANCHOR_FILTER = 0.70
LEFT_ANCHOR_MEMORY_FRAMES = 6
LEFT_ANCHOR_ERROR_LIMIT = 80.0
LATERAL_SAFETY_START_RATIO = 0.08
LATERAL_SAFETY_GAIN = 0.15
LATERAL_SAFETY_MAX = 10.0

# 차량에서 오른쪽 외곽선까지 남은 폭이 전체 차선 폭의 이 비율보다 작으면
# 왼쪽 복귀를 추가합니다. 절대 X좌표를 쓰지 않아 S자의 원근 변화에 대응합니다.
#
# (2026-08-04 지그재그 원인 분석 후 수정) 기존에는 RIGHT_SAFE_CLEARANCE
# (soft, guard 시작점)와 BOUNDARY_HARD_CLEARANCE(hard, 강제조향 시작점)
# 사이 여유가 차로폭 300~400px 기준 약 15px밖에 안 됐습니다. 그 좁은
# 구간 안에서 guard가 낼 수 있는 최대 보정력은 RIGHT_GUARD_MAX(40)의
# 약 24%뿐이었고, 결국 대부분 guard가 부족한 채로 hard(강제 12도 오프셋,
# 빠른 조향속도)로 넘어가 버렸습니다. hard는 원래 세게/빠르게 튕겨내도록
# 설계돼 있어(S자에서 필요), 이게 반복되면 반대쪽 경계까지 넘어가
# 다시 hard가 걸리는 좌우 진동(지그재그)이 만들어졌습니다
# (실차 로그: error가 +50대/-30대를 5~7프레임 주기로 반복, hard가
# -1/0/1로 계속 뒤바뀜을 확인).
#
# BOUNDARY_HARD_CLEARANCE_* 는 S자 통과에 필요해 그대로 두고, 대신
# RIGHT_SAFE_CLEARANCE_*(soft 시작을 더 일찍)와 RIGHT_GUARD_GAIN/MAX
# (그 구간에서 더 세게 반응)를 올려 guard가 hard 발동 전에 이미 최대
# 보정력을 다 쓸 수 있게 여유폭을 15px -> 60px로 넓혔습니다. 목표는
# '평소엔 guard만으로 복귀, hard는 S자처럼 정말 급할 때만 발동'입니다.
# 실차에서 재검증 필요.
RIGHT_SAFE_CLEARANCE_RATIO = 0.60          # 0.48 -> 0.60 (guard를 더 일찍 시작)
RIGHT_SAFE_CLEARANCE_MIN = 230.0           # 185.0 -> 230.0
RIGHT_GUARD_DEADBAND = 3.0
# [2026-08-06 01:xx Claude 수정] 1.30 -> 0.79. soft(230)~hard(170) 여유폭
# 60px 전체를 0->45로 매끄럽게 채우려면 gain=45/(60-3)≈0.79가 필요한데,
# 1.30이면 여유폭이 63% 남은 시점(clearance=192.4)에서 이미 45로
# 포화돼 그 뒤로는 실제 여유와 무관하게 45 고정 출력(on/off 릴레이)이
# 된다. 실차 로그(20260806-0312-19, 8초 부근)에서 guard=45 고정 +
# error가 -150~+42 왕복하는 패턴으로 재확인. 8/5에 이미 계산/설명했으나
# 실제 파일에는 반영이 안 돼 있었다.
RIGHT_GUARD_GAIN = 0.79                    # 1.30 -> 0.79
RIGHT_GUARD_MAX = 45.0                     # 40.0 -> 45.0
BOUNDARY_GUARD_MAX_Y_GAP_RATIO = 0.20
BOUNDARY_HARD_CLEARANCE_RATIO = 0.44       # 그대로 (S자 대응력 보존)
BOUNDARY_HARD_CLEARANCE_MIN = 170.0        # 그대로
BOUNDARY_STEER_GAIN = 0.22
BOUNDARY_STEER_MAX = 10.0
BOUNDARY_STEER_FILTER = 0.84
BOUNDARY_HARD_STEER_OFFSET = 12.0          # 그대로 (S자 대응력 보존)
BOUNDARY_HARD_STEER_RATE = 130.0           # 그대로
BOUNDARY_HARD_CONFIRM_FRAMES = 2
INFERRED_BOUNDARY_GUARD_SCALE = 0.50
# [2026-08-06 Claude 추가] 경계 근처에서 preview(전방주시) 비중을 줄일 때의
# 하한. 예전엔 하한이 0이라 hard_boundary가 켜지는 순간 preview_weight가
# 통째로 0이 됐고, 그러면 제어기가 범퍼 바로 앞 한 점만 보고 조향하게 돼
# 속도가 붙으면 반드시 발진했다(실차 20260806-0425-49, 71~81초에서 진폭이
# 88->149->192로 자라는 발산 확인. 그 구간 73%에서 PW=0.00이었다).
# 자세한 메커니즘은 lane_controller.py의 left_guard 쪽 주석 참고.
# 판정: 진동 진폭이 줄면 성공. 반대로 경계를 더 자주 넘으면(차선 물면)
#   preview가 과하게 살아난 것이므로 0.3 정도로 낮출 것.
BOUNDARY_PREVIEW_MIN_SCALE = 0.5

# 조향 안정화 설정
ERROR_FILTER = 0.68
DERIVATIVE_FILTER = 0.82
STEER_RIGHT = 100
STEER_LEFT = 210
STEER_KP = 0.17
# [2026-08-06 Claude 수정] 0.004 -> 0.015. 실차 로그(20260806-0339-30,
# 34~40초, 4초 주기로 error가 -91~+76 왕복)에서 그 진동의 실제 변화율
# (~85px/s)로 계산하면 미분항 기여가 85*0.004=0.34 PWM으로 사실상 0.
# 미분항이 이름만 있고 실제로는 진동 억제에 전혀 기여를 못 하고 있었다.
# 한 번에 목표치(0.06~0.12 추정)까지 올리면 잡음 증폭 위험이 커서
# 4배 정도(0.015)만 먼저 올려 방향을 본다.
# 판정: 진폭/주기가 줄면 성공. 새로운 고주파 떨림이 생기면 과도한
# 것이니 0.008~0.010 사이로 낮출 것.c
STEER_KD = 0.06
STEER_DEADBAND = 4.0
STEER_RATE_PER_SECOND = 95.0
HIGH_SPEED_STEER_RATE_REDUCTION = 0.20
MEMORY_STEER_RETAIN_RATIO = 0.45

# S자에서는 정확도를 우선해 직선보다 자동으로 감속합니다.
BASE_SPEED = 180
CURVE_SPEED = 125
CURVE_MAX_SPEED = 130
CURVE_SPEED_TRIGGER = 18.0
RETURN_SPEED = 105
MIN_EFFECTIVE_DRIVE_SPEED = 105
DRIVE_START_BOOST_SPEED = 120
DRIVE_START_BOOST_SECONDS = 0.20
ERROR_SPEED_GAIN = 0.20
HEADING_SPEED_GAIN = 0.20
CURVATURE_SPEED_GAIN = 0.16

# 화면 표시 및 로그 출력 설정
WINDOW_NAME = "Autonomous Car"
SHOW_DEBUG = True
PRINT_INTERVAL = 10

# -------------------------------------------------------------------
# 라이다 장애물 감지 (obstacle_detector.py)
# -------------------------------------------------------------------
# 아두이노/카메라와 별개로 라이다만 연결하는 시리얼 포트입니다.
# 장치관리자(윈도우) 또는 `python -m serial.tools.list_ports`로 확인하세요.
LIDAR_ENABLED = True
LIDAR_PORT = "COM3"

# 라이다 스캔 각도 중 "차량 정면"에 해당하는 값(도)입니다. 라이다 장착
# 방향에 따라 다르므로 반드시 calibrate_lidar_angle.py로 실차에서 먼저
# 확인한 뒤 이 값을 바꿔주세요. 기본값 0은 임시값입니다.
LIDAR_FRONT_ANGLE = 0.0

# [2026-08-06 Claude 수정] 트랙이 2차선이고 우리는 바깥쪽 2차선만 달리므로,
# 옆(1차선) 차선의 장애물까지 잡으면 안 된다는 요구사항이 있었습니다.
# 기존에는 "정면 기준 ±N도"라는 순수 각도창으로 판단했는데, 이 방식은
# 두 차선이 원근 때문에 거리가 멀어질수록 각도차가 점점 좁아져(소실점
# 효과) 어느 거리부턴 각도만으로 내 차선/옆 차선을 구분할 수 없다는
# 근본적 한계가 있습니다.
#
# 그래서 각도창 대신, 라이다 점을 차량 진행축 기준 직교좌표로 바꿔
# "횡방향 거리(진행축에서 좌우로 얼마나 떨어졌나)가 내 차선 폭 절반
# 이내인가"로 판정하도록 obstacle_detector.py를 바꿨습니다
# (LidarObstacleDetector._hit_in_lane_corridor 참고). 이 방식은 거리에
# 무관하게 항상 "내 차선 폭"만큼만 보므로 원근 문제가 없습니다.
#
# [2026-08-06 Claude 수정 2] 트랙 전체 폭 기준 좌표로 다시 정리했습니다.
# 트랙을 가로지르는 선 3개를 안쪽부터 0cm(1차선 안쪽 경계) / 34cm(1차선과
# 2차선 사이 중앙선) / 68cm(2차선 바깥쪽 경계, 트랙 전체 끝)라고 하면,
# 우리가 달리는 2차선은 34~68cm 구간(폭 34cm)이고 그 중앙은 51cm입니다.
# 라이다가 차체 중앙에 고정돼 있고(범퍼 아래 판을 덧대 그 앞에 장착,
# 좌우 오프셋 없음) 차량이 2차선 중앙을 달린다고 가정하면, 라이다 기준
# lateral=0mm는 이 51cm 지점과 같습니다.
#
# LIDAR_LANE_WIDTH_MM은 우리가 달리는 2차선 하나의 폭입니다(340mm).
# 실측치가 바뀌거나 라이다가 차체 중앙에서 벗어나 있으면 이 값과 아래
# 오프셋 가정을 함께 고쳐야 합니다. 이 값 자체는 판정에 직접 쓰이지
# 않고, 아래 LIDAR_OBSTACLE_LATERAL_*_MM을 정하기 위한 참고 치수입니다.
LIDAR_LANE_WIDTH_MM = 340.0

# 트랙 전체 좌표(안쪽 0cm / 중앙선 34cm / 바깥 68cm) 기준으로 감지할
# 횡방향 구간입니다. 2차선 중앙(차량 정중앙)은 트랙 51cm이고, lateral
# (라이다 기준 mm)는 (트랙_cm - 51) * 10 으로 환산합니다.
# 예: 40cm -> -110mm, 64cm -> +130mm
LIDAR_TRACK_LANE2_CENTER_CM = 51.0
LIDAR_OBSTACLE_TRACK_MIN_CM = 40.0
LIDAR_OBSTACLE_TRACK_MAX_CM = 64.0
LIDAR_OBSTACLE_LATERAL_MIN_MM = (
    LIDAR_OBSTACLE_TRACK_MIN_CM - LIDAR_TRACK_LANE2_CENTER_CM
) * 10.0
LIDAR_OBSTACLE_LATERAL_MAX_MM = (
    LIDAR_OBSTACLE_TRACK_MAX_CM - LIDAR_TRACK_LANE2_CENTER_CM
) * 10.0

# 1m 50cm 이내만 "장애물"로 판단합니다. 라이다 값은 mm 단위입니다.
LIDAR_DETECT_MIN_DISTANCE_MM = 50.0
LIDAR_DETECT_MAX_DISTANCE_MM = 1500.0
# 정면 각도(LIDAR_FRONT_ANGLE) 미세 오차가 멀수록 lateral(mm)로
# 커집니다. 1.5m에서 ±3°면 약 ±78mm — 이 비율만큼 거리에 비례해
# 감지 폭을 살짝 넓혀 먼 거리(150cm)에서도 잡히게 합니다.
LIDAR_LATERAL_DISTANCE_SLACK_RATIO = 0.04

# 노이즈로 인한 오검출을 막기 위해, 연속 스캔에서 이 횟수 이상 감지되어야
# "장애물 있음"으로 확정합니다.
LIDAR_DETECT_CONFIRM_COUNT = 2

# 라이다 스캔 스레드가 이 시간(초) 동안 새 데이터를 못 주면 통신이 끊긴
# 것으로 보고 안전하게 "장애물 없음"으로 취급합니다(정지가 아니라 무시).
LIDAR_STALE_SECONDS = 0.5

# -------------------------------------------------------------------
# 장애물 회피(카메라 기반 차선 오프셋) 기동 (obstacle_detector.py: AvoidanceController)
# -------------------------------------------------------------------
# [2026-08-06 Claude 수정 3] 기존에는 "정해진 각도로 정해진 시간만 꺾는"
# 오픈루프(카메라 무시) 방식으로 OUT(1차선 진입)->HOLD(직진 통과)->
# RETURN(2차선 복귀) 3단계를 시간으로만 관리했습니다. 실차에서 이
# 방식은 각도/시간 조합에 아주 민감해서(1.4초는 부족, 4.0초+각도35는
# 과회전으로 카메라가 트랙 밖을 보는 등) 계속 재조정이 필요했습니다.
#
# 그래서 방식을 바꿨습니다: 이제 AvoidanceController는 조향을 직접
# 계산하지 않고, LaneController.set_lane_offset()으로 "목표 차선을
# 옆으로 옮겨라"라고만 지시합니다. 실제 조향은 기존 카메라 차선 추종
# PID(_calculate_control)가 매 프레임 계속 담당합니다 — 즉 옆 차선에
# 정확히 안착할 때까지 카메라 피드백이 계속 작동하고, 각도/시간을
# 따로 맞출 필요가 없습니다. reset_tracking()도 더 이상 필요 없습니다
# (오픈루프로 카메라를 무시하는 구간이 없어져서, 차선 인식이 한 번도
# 끊기지 않습니다).
#
# [2026-08-06 Claude 수정 5] 시간 기반 로직(AVOID_DURATION_SECONDS,
# AVOID_COOLDOWN_SECONDS)을 완전히 없앴습니다. 자동 복귀도 없고, 장애물을
# 새로 감지할 때마다(상승 에지) 옆 차선으로 토글합니다. 감지가 잠깐
# 끊겼다 다시 잡혀도 AVOID_TOGGLE_HOLD_SECONDS 안에는 토글하지 않아
# 차선 변경 도중 목표가 원위치로 돌아가지 않게 합니다.
#
# 주의: 라이다 감지 코리더는 차량 진행축 기준 고정 폭이라, 회피 조향을
# 시작하면 장애물이 코리더 밖으로 나가 감지가 꺼질 수 있습니다. 자동
# 복귀는 하지 않으므로 오프셋은 유지되지만, 감지 깜빡임으로 토글이
# 연속 발생하면 목표가 왔다 갔다 할 수 있어 홀드 시간이 필요합니다.
AVOID_LANE_OFFSET_LANES = 1.0

# +1: STEER_LEFT 방향(왼쪽 차선, 1차선)으로 회피, -1: STEER_RIGHT
# 방향(오른쪽 차선)으로 회피. 트랙에서 1차선이 어느 쪽인지에 맞게
# 바꾸세요. 부호만 쓰이고(LaneController.set_lane_offset의 방향 결정),
# 복귀는 오프셋을 0으로 되돌리기만 하면 되므로 별도 반대방향 설정이
# 필요 없습니다.
AVOID_LANE_DIRECTION = 1

# 회피 중(오픈루프 종료 후 카메라 추종 포함) 속도 상한입니다.
AVOID_SPEED = 120

# 오픈루프 차선 변경: 최대 조향으로 옆 차선 이동 후 반대 조향으로 자세 보정.
AVOID_OPEN_LOOP_ENABLED = True
AVOID_OPEN_LOOP_SPEED = 120
AVOID_OPEN_LOOP_OUT_SECONDS = 1.5
AVOID_OPEN_LOOP_COUNTER_SECONDS = 0.4
# 1차선(왼쪽)으로 갈 때 OUT=STEER_LEFT, COUNTER=STEER_RIGHT 입니다.
AVOID_OPEN_LOOP_STEER_LEFT = STEER_LEFT
AVOID_OPEN_LOOP_STEER_RIGHT = STEER_RIGHT

# 장애물 감지 토글 후 이 시간(초) 동안은 감지가 잠깐 끊겼다 다시 잡혀도
# 목표 차선을 다시 토글하지 않습니다. 차선 변경이 끝나기 전에 라이다
# 감지가 깜빡이면 0<->1 오프셋이 왔다 갔다 하며 회피가 중간에 멈출 수
# 있습니다.
AVOID_TOGGLE_HOLD_SECONDS = 3.0