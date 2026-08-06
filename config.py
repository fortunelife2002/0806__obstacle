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
LANE_WHITE_VALUE_MIN = 175
LANE_WHITE_SATURATION_MAX = 65
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
LIDAR_PORT = "COM5"

# 라이다 스캔 각도 중 "차량 정면"에 해당하는 값(도)입니다. 라이다 장착
# 방향에 따라 다르므로 반드시 calibrate_lidar_angle.py로 실차에서 먼저
# 확인한 뒤 이 값을 바꿔주세요. 기본값 0은 임시값입니다.
LIDAR_FRONT_ANGLE = 0.0

# 정면 각도 기준 ±LIDAR_DETECT_HALF_WINDOW_DEG 범위만 봅니다.
# (예: 5.0이면 정면 기준 좌우 5도씩, 총 10도 폭)
LIDAR_DETECT_HALF_WINDOW_DEG = 5.0

# 1m 30cm 이내만 "장애물"로 판단합니다. 라이다 값은 mm 단위입니다.
LIDAR_DETECT_MIN_DISTANCE_MM = 50.0
LIDAR_DETECT_MAX_DISTANCE_MM = 1300.0

# 노이즈로 인한 오검출을 막기 위해, 연속 스캔에서 이 횟수 이상 감지되어야
# "장애물 있음"으로 확정합니다.
LIDAR_DETECT_CONFIRM_COUNT = 2

# 라이다 스캔 스레드가 이 시간(초) 동안 새 데이터를 못 주면 통신이 끊긴
# 것으로 보고 안전하게 "장애물 없음"으로 취급합니다(정지가 아니라 무시).
LIDAR_STALE_SECONDS = 0.5

# -------------------------------------------------------------------
# 장애물 회피(옆 차선 이동) 기동 (obstacle_detector.py: AvoidanceController)
# -------------------------------------------------------------------
# 장애물 감지 즉시, 차선 인식 결과를 무시하고 이 조향값/속도로 정해진
# 시간 동안 옆 차선으로 이동합니다(카메라 피드백 없는 오픈루프 기동이라
# 실차에서 AVOID_STEER_OFFSET / AVOID_DURATION_SECONDS를 반드시 튜닝해야
# 실제로 옆 차선 폭만큼만 이동합니다).
AVOID_STEER_OFFSET = 35

# +1: STEER_LEFT 방향(왼쪽 차선)으로 회피, -1: STEER_RIGHT 방향(오른쪽
# 차선)으로 회피. 트랙에서 옆 차선이 어느 쪽인지에 맞게 바꾸세요.
AVOID_LANE_DIRECTION = 1

AVOID_DURATION_SECONDS = 1.4
AVOID_SPEED = 100

# 회피 기동이 끝난 뒤, 같은 장애물에 다시 반응해 좌우로 흔들리지 않도록
# 이 시간(초) 동안은 재감지가 있어도 무시하고 일반 차선 추종을 유지합니다.
AVOID_COOLDOWN_SECONDS = 3.0