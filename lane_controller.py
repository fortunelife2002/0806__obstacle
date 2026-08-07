"""다중 높이 차선 중앙점을 이용한 S자 곡선 추종 제어기입니다.

기존 방식은 가까운 구간과 먼 구간에서 각각 한 점만 선택했기 때문에
S자의 방향이 바뀌는 구간을 안정적으로 표현하기 어려웠습니다. 이 제어기는
ROI의 여러 높이에서 좌우 차선을 찾고, 중앙점들을 3차 곡선으로 연결하여
가까운 경로와 앞으로 이어질 경로를 동시에 따라갑니다.

[2026-08-06 01:17 Claude 수정] 박스형 표시(주차 테스트 칸) 오검출 대응으로
_make_mask 끝에 박스 제거 단계를 추가하고, 아래 세 메서드를 새로
만들었습니다(기존 로직은 건드리지 않았습니다).
  _remove_box_structures     - 마스크에서 박스 영역을 지우는 진입점
  _confirm_horizontal_bars   - 가로 막대 후보를 모양(방향/두께/직선성)으로 검증
  _group_bars_into_ladders   - 나란한 막대 2개 이상만 박스로 인정
검증: 녹화 영상 한 바퀴(OR-stOLS1b08BZzYxPzudg.mp4, 1829프레임)를 필터
on/off로 각각 재생 비교. LOST 122프레임으로 동일, TRACKING 556->561.
급커브 차선과 배경(나무바닥)은 한 픽셀도 안 지우고, 박스 세로분할선만
1336->493픽셀로 줄었습니다. 자세한 시행착오는 각 메서드 docstring 참고.
"""

import time

import cv2
import numpy as np

import config as cfg


def clamp(value, minimum, maximum):
    """값을 지정한 범위 안으로 제한합니다."""
    return max(minimum, min(value, maximum))


def scale_x(value, width):
    """640픽셀 기준 X좌표를 현재 영상 너비에 맞게 변환합니다."""
    return value * width / cfg.REFERENCE_WIDTH


class LaneController:
    """여러 높이의 차선 중앙 경로로 속도와 조향값을 계산합니다."""

    def __init__(self):
        self.path_coefficients = None
        self.filtered_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_error = 0.0
        self.steer_float = float(cfg.STEER_CENTER)
        self.previous_steer = cfg.STEER_CENTER
        self.speed_float = float(cfg.BASE_SPEED)
        self.previous_speed = cfg.BASE_SPEED
        self.lost_count = 0
        # 차선 폭은 가까울수록 넓어지는 원근 모델(1차식)로 저장합니다.
        # 값은 화면 너비로 정규화하므로 해상도가 달라져도 그대로 동작합니다.
        self.width_coefficients = None
        self.width_model_age = cfg.SINGLE_LANE_WIDTH_MAX_AGE + 1
        self.last_update_time = None
        self.valid_streak = 0
        self.path_locked = False
        self.single_lane_side = None
        self.single_lane_frames = 0
        self.corridor_steer_state = 0.0
        self.right_hard_frames = 0
        self.left_hard_frames = 0
        self.curve_target_offset_state = 0.0
        self.left_anchor_error_state = 0.0
        self.left_anchor_age = cfg.LEFT_ANCHOR_MEMORY_FRAMES + 1
        self.center_missing_rows = 0
        # [2026-08-06 Claude 추가] 라이다 장애물 회피용 차선 오프셋(단위:
        # 학습된 차선 폭의 배수). 0.0=평소처럼 지금 차선 중앙을 목표로
        # 하고, 양수/음수면 obstacle_detector.AvoidanceController가
        # set_lane_offset()으로 옆 차선 목표를 지시한 상태입니다.
        # reset_tracking()에서는 안 지웁니다 — 프레임 유실 등으로 경로
        # 기억이 지워져도 "지금 회피 중이다"라는 상태 자체는 유지되어야
        # 하기 때문입니다.
        self._lane_offset_lanes = 0.0

    def set_lane_offset(self, lanes):
        """장애물 회피 중 목표 차선을 바꿉니다.

        lanes=0.0이면 지금 카메라가 인식하고 있는 차선(보통 2차선) 중앙을
        그대로 목표로 삼습니다. lanes=+1.0이면 config.AVOID_LANE_DIRECTION
        방향으로 차선 폭 한 개만큼 목표를 옮겨서(옆 차선, 1차선), 실제
        조향은 이 목표를 향해 매 프레임 카메라 제어 루프(_calculate_control)
        가 계속 보정합니다. 기존처럼 "정해진 각도로 정해진 시간만 꺾는"
        오픈루프 방식과 달리, 라인 검출/피팅은 전혀 바꾸지 않고 최종
        목표 위치만 옮기므로 실제로 그 차선에 안착할 때까지 카메라
        피드백이 계속 작동합니다.
        """
        self._lane_offset_lanes = float(lanes)

    def reset_tracking(self):
        """카메라가 끊긴 뒤 오래된 경로와 조향 기억을 안전하게 지웁니다."""
        self.path_coefficients = None
        self.width_coefficients = None
        self.width_model_age = cfg.SINGLE_LANE_WIDTH_MAX_AGE + 1
        self.filtered_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_error = 0.0
        self.steer_float = float(cfg.STEER_CENTER)
        self.previous_steer = cfg.STEER_CENTER
        self.speed_float = float(cfg.BASE_SPEED)
        self.previous_speed = 0
        self.lost_count = 0
        self.last_update_time = None
        self.valid_streak = 0
        self.path_locked = False
        self.single_lane_side = None
        self.single_lane_frames = 0
        self.corridor_steer_state = 0.0
        self.right_hard_frames = 0
        self.left_hard_frames = 0
        self.curve_target_offset_state = 0.0
        self.left_anchor_error_state = 0.0
        self.left_anchor_age = cfg.LEFT_ANCHOR_MEMORY_FRAMES + 1
        self.center_missing_rows = 0

    @staticmethod
    def _make_mask(frame):
        """영상 아래쪽에서 흰색 차선 후보 마스크를 만듭니다."""
        bottom = min(cfg.ROI_BOTTOM, frame.shape[0])
        top = min(cfg.ROI_TOP, bottom - 1)
        roi = frame[top:bottom, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        # 흰색은 밝으면서 채도가 낮습니다. 사진의 초록색 바닥은 밝지만
        # 채도가 높기 때문에 이 조건에서 제외됩니다.
        # (채도 하한선으로 바닥 반사광을 거르는 시도를 했었지만, 조명 조건이
        # 바뀌면 진짜 흰 선의 채도도 함께 낮아져서 오히려 선을 통째로
        # 못 보게 되는 문제가 있어 되돌렸습니다.)
        white = (
            (value >= cfg.LANE_WHITE_VALUE_MIN)
            & (saturation <= cfg.LANE_WHITE_SATURATION_MAX)
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
        if cfg.BOX_FILTER_ENABLED:
            mask = LaneController._remove_box_structures(mask)
        return roi, mask, cfg.LANE_WHITE_VALUE_MIN

    @staticmethod
    def _remove_box_structures(mask):
        """가로 테두리를 가진 박스형 표시(주차 테스트 칸 등)를 통째로 지웁니다.

        [2026-08-05] 신규. 실차 영상(20260805-1032-45, 44초 부근)에서
        박스 내부의 세로 분할선이 진짜 차선(점선/실선)처럼 잡혀 조향이
        그쪽으로 끌려가는 오검출을 확인했다. _row_segments는 한 행 폭만
        보고 판단해서 박스의 세로 분할선과 진짜 차선을 구분하지 못한다.

        1차 시도(연결요소 단위로 모양 판정, just_run의 horizontal_
        structure와 같은 방식)는 실패했다 - 박스의 위/아래 가로변과
        세로변, 내부 분할선이 모서리에서 서로 맞닿아 connectedComponents
        가 전부 하나의 "사다리" 모양 덩어리로 묶어버려서, 그 덩어리
        전체의 종횡비로는 "가로로 길쭉하다"는 조건을 통과하지 못했다
        (실제 raw 프레임으로 검증: 제거된 픽셀이 0에 가까웠음).

        2차 시도(가로 방향 모폴로지 열기로 찾은 픽셀 주변만 지우기)도
        절반만 성공했다 - 가로 변/분할선 자체는 지워지지만, 그것과
        맞닿은 세로변(박스 좌우 테두리, 문제의 내부 분할선 아래쪽 꼬리)
        은 마진 밖으로 남았다(실측: 박스 영역에 3010픽셀 잔존).

        최종: 두 방법을 합친다. 가로 방향 열기로 "박스의 일부인 게
        확실한" 픽셀을 찾는 건 그대로 쓰되, 그 픽셀과 8-연결로 이어진
        원본 마스크의 연결요소 전체(세로변·분할선 포함)를 통째로 지운다.
        박스는 모든 변이 서로 맞닿은 하나의 연결요소이므로 이렇게 하면
        박스 전체가 지워진다. 진짜 차선(점선 조각도 폭
        TRACK_MAX_SEGMENT_WIDTH=45px 이내)은 어느 한 행에서 커널 폭만큼
        길게 이어지지 않으므로 가로 열기에서 살아남는 픽셀이 전혀 없고,
        따라서 그 연결요소도 지워지지 않는다.

        [2026-08-05 3차 수정 - 위 전제가 틀렸다] 위 "진짜 차선은 한 행에서
        커널 폭만큼 이어지지 않는다"는 전제가 실차 데이터에서 깨졌다.
        급커브 정점에서는 차선이 실제로 가로로 눕는다. 전체 랩 재생
        검증(OR-stOLS1b08BZzYxPzudg.mp4)에서 11.0초의 급커브 트랙
        경계선이 통째로 지워지는 걸 확인했다(9248픽셀 제거, 그 덩어리의
        최대 가로 연속 길이가 정확히 81px이라 80px 커널에 걸렸다).
        그 여파로 내부 상태(경로 계수/차폭 모델/메모리)가 어긋나면서
        13~19초, 31~37초에서 LOST가 폭증했다(전체 LOST 122 -> 585 프레임).

        그래서 "가로 연속 길이"만으로 판정하던 걸 버리고, 열기로 찾은
        가로 막대 후보마다 아래 세 조건을 모두 요구한다. 실측으로
        박스 가로변과 급커브 차선이 이 세 지표에서 확연히 갈린다:
                           박스 가로변(t=44)   급커브 차선(t=11)
          방향 |vy|         0.008 / 0.015      0.988
          두께 h            9px                123px
          직선성 잔차std    2.6 / 3.0          13.7
        커브는 정점에서 국소적으로만 가로일 뿐 덩어리 전체의 방향은
        수직이고(|vy|이 1에 가깝다) 휘어 있다(잔차가 크다). 반면 박스
        가로변은 얇고 곧고 진짜 수평이다.

        [2026-08-06 근접 구간 보정 시도했다가 되돌림] 위 판정은 박스의
        가로 변(위/아래 테두리)이 ROI 안에 둘 다 보여야 작동한다. 실차
        영상(20260806-0312-19, 22~24초)에서 차가 박스에 바짝 붙으면
        가로 테두리가 ROI 위쪽으로 잘려나가 짝지을 막대가 아예 없어지고,
        세로 분할선만 남아 다시 차선으로 오검출되는 걸 확인했다.

        그래서 사다리를 확실히 찾은 프레임에서 그 x범위를 잠시(수십
        프레임) 기억해뒀다가, 이후 사다리를 다시 못 찾아도 그 x범위를
        [기억한 위쪽 y좌표, 화면 아래끝]까지 계속 지우는 시도를 했다.
        전체 랩 재생 비교(OR-stOLS1b08BZzYxPzudg.mp4)에서 심각한 부작용이
        나왔다: LOST가 122->308로 급증했고, 박스와 무관한 20.5초 지점부터
        이미 사다리-only 버전과 어긋나기 시작했다. 원인으로 추정되는 것:
        아주 드물게 다른 곳(체육관 바닥 이음새 등)에서 사다리가 잘못
        확인되면, 사다리-only 버전은 그 프레임 하나만 지우고 끝나지만
        기억 버전은 그 오탐 위치를 화면 맨 아래까지 수십 프레임 동안
        계속 지워버려서 진짜 차선이 있는 근접 구간을 통째로 날린다.
        "아래로 확장"이 안전한 방향이라는 전제가, 오탐이 섞이면 오히려
        피해를 증폭시키는 쪽으로 작용했다.

        박스 조각이 남는 것보다 훨씬 위험해서 기억 없이 매 프레임 새로
        판정하는 버전(아래 코드)으로 되돌렸다. 근접 구간에서 세로
        분할선이 남는 문제는 아직 미해결이다 - 다시 시도한다면 "화면
        아래까지 무조건 확장"이 아니라 오탐 시 피해가 제한되는 다른
        방식(예: 여러 프레임 연속 확인 후에만 기억 시작, 기억 영역
        높이를 제한)을 실차 영상 여러 개로 먼저 검증해야 한다.
        """
        width = mask.shape[1]
        kernel_w = int(round(
            scale_x(cfg.BOX_FILTER_HORIZONTAL_KERNEL_WIDTH, width)
        ))
        kernel_w = max(3, kernel_w)
        horizontal_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_w, 1)
        )
        horizontal_bars = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, horizontal_kernel
        )
        if not horizontal_bars.any():
            return mask

        bars = LaneController._confirm_horizontal_bars(
            horizontal_bars, width
        )
        ladders = LaneController._group_bars_into_ladders(bars, width)
        if not ladders:
            return mask

        x_margin = max(1, int(round(
            scale_x(cfg.BOX_FILTER_X_MARGIN, width)
        )))
        y_margin = max(1, int(round(
            scale_x(cfg.BOX_FILTER_Y_MARGIN, width)
        )))
        cleaned = mask.copy()
        height = mask.shape[0]
        for x0, x1, y0, y1 in ladders:
            cleaned[
                max(0, y0 - y_margin):min(height, y1 + y_margin),
                max(0, x0 - x_margin):min(width, x1 + x_margin),
            ] = 0
        return cleaned

    @staticmethod
    def _group_bars_into_ladders(bars, width):
        """가로 막대들을 "사다리"(=박스)로 묶습니다.

        [2026-08-05] 박스의 진짜 특징은 가로 변이 최소 2개(위/아래
        테두리) 나란히 있고 서로 x방향으로 겹친다는 것이다. 실차
        영상에서 밝은 체육관 나무 바닥의 이음새 선도 "얇고 곧고 수평"
        이라 막대 하나로는 박스와 구분이 안 됐고, 그 선에 연결된 거대한
        배경 덩어리가 통째로 지워지면서(프레임당 최대 3만 픽셀) 추적
        상태가 크게 흔들렸다. 나란한 막대 2개를 요구하면 단일 배경
        선은 자연히 제외된다. 삭제도 연결요소 전체가 아니라 사다리의
        바깥 사각형으로만 제한해서, 우연히 배경과 이어져 있어도
        번지지 않게 한다.
        """
        if not bars:
            return []
        min_gap = scale_x(cfg.BOX_FILTER_LADDER_MIN_GAP, width)
        max_gap = scale_x(cfg.BOX_FILTER_LADDER_MAX_GAP, width)
        ladders = []
        used = set()
        for i, first in enumerate(bars):
            if i in used:
                continue
            group = [first]
            for j, second in enumerate(bars):
                if j <= i or j in used:
                    continue
                gap = abs(second["cy"] - first["cy"])
                if not (min_gap <= gap <= max_gap):
                    continue
                overlap = (
                    min(first["x1"], second["x1"])
                    - max(first["x0"], second["x0"])
                )
                shorter = min(
                    first["x1"] - first["x0"],
                    second["x1"] - second["x0"],
                )
                if shorter <= 0:
                    continue
                if overlap / float(shorter) < cfg.BOX_FILTER_LADDER_MIN_OVERLAP:
                    continue
                group.append(second)
                used.add(j)
            if len(group) < 2:
                continue
            used.add(i)
            ladders.append((
                min(b["x0"] for b in group),
                max(b["x1"] for b in group),
                min(b["y0"] for b in group),
                max(b["y1"] for b in group),
            ))
        return ladders

    @staticmethod
    def _confirm_horizontal_bars(horizontal_bars, width):
        """가로 막대 후보 중 진짜 박스 변인 것만 남깁니다.

        [2026-08-05] _remove_box_structures 참고. 가로 방향 열기만으로는
        급커브 정점의 차선(그 순간 가로로 눕는다)까지 잡히므로, 후보마다
        모양을 재서 걸러낸다. 박스 변은 얇고 곧고 진짜 수평인 반면,
        커브 차선은 덩어리 전체 방향이 수직이고 휘어 있다.
        """
        n, labels, stats, _ = cv2.connectedComponentsWithStats(
            horizontal_bars, connectivity=8
        )
        confirmed = []
        max_residual = scale_x(cfg.BOX_FILTER_MAX_BAR_RESIDUAL, width)
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < cfg.BOX_FILTER_MIN_BAR_AREA:
                continue
            if h <= 0 or w / float(h) < cfg.BOX_FILTER_MIN_BAR_ASPECT:
                continue
            component = (labels[y:y + h, x:x + w] == i).astype(
                np.uint8
            ) * 255
            contours, _ = cv2.findContours(
                component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            vx, vy, x0, y0 = cv2.fitLine(
                contour, cv2.DIST_L2, 0, 0.01, 0.01
            )
            if abs(float(vy[0])) > cfg.BOX_FILTER_MAX_BAR_VY:
                continue
            points = contour.reshape(-1, 2).astype(np.float64)
            normal_x, normal_y = -float(vy[0]), float(vx[0])
            residual = (
                (points[:, 0] - float(x0[0])) * normal_x
                + (points[:, 1] - float(y0[0])) * normal_y
            )
            if float(np.std(residual)) > max_residual:
                continue
            confirmed.append({
                "x0": x, "x1": x + w,
                "y0": y, "y1": y + h,
                "cy": y + h * 0.5,
            })
        return confirmed

    @staticmethod
    def _forward_value(y, height):
        """ROI 아래쪽을 0, 위쪽을 1로 바꾼 전방 거리값입니다."""
        return 1.0 - y / max(1.0, float(height - 1))

    @staticmethod
    def _default_boundaries(y_ratio, width):
        """첫 프레임에서 사용할 원래 코드의 차선 위치를 보간합니다."""
        near = cfg.LANE_BANDS[0]
        far = cfg.LANE_BANDS[1]
        near_y = sum(near["y"]) * 0.5
        far_y = sum(far["y"]) * 0.5
        denominator = max(0.01, near_y - far_y)
        ratio = (y_ratio - far_y) / denominator
        left = far["left"] + ratio * (near["left"] - far["left"])
        right = far["right"] + ratio * (near["right"] - far["right"])
        expected_width = clamp(
            right - left,
            cfg.TRACK_MIN_LANE_WIDTH,
            cfg.TRACK_MAX_LANE_WIDTH,
        )
        center = (left + right) * 0.5
        return scale_x(center, width), scale_x(expected_width, width)

    @staticmethod
    def _row_segments(mask, y):
        """한 높이 주변에서 흰색 덩어리들의 X좌표와 강도를 찾습니다."""
        half = cfg.TRACK_STRIP_HALF_HEIGHT
        y1 = max(0, int(y) - half)
        y2 = min(mask.shape[0], int(y) + half + 1)
        strip = mask[y1:y2, :]
        if strip.size == 0:
            return []

        counts = np.count_nonzero(strip, axis=0)
        required = max(
            2,
            int(strip.shape[0] * cfg.TRACK_COLUMN_MIN_RATIO),
        )
        active = counts >= required
        changes = np.diff(
            np.concatenate(([False], active, [False])).astype(np.int8)
        )
        starts = np.where(changes == 1)[0]
        ends = np.where(changes == -1)[0]

        segments = []
        for start, end in zip(starts, ends):
            segment_width = int(end - start)
            if not (
                cfg.TRACK_MIN_SEGMENT_WIDTH
                <= segment_width
                <= cfg.TRACK_MAX_SEGMENT_WIDTH
            ):
                continue
            weights = counts[start:end].astype(np.float64)
            xs = np.arange(start, end, dtype=np.float64)
            total = float(weights.sum())
            if total <= 0:
                continue
            segments.append({
                "x": float(np.dot(xs, weights) / total),
                "strength": total,
                "width": segment_width,
            })
        return segments

    def _previous_center(self, y, height, default_center):
        if self.path_coefficients is None:
            return default_center
        forward = self._forward_value(y, height)
        return float(np.polyval(self.path_coefficients, forward))

    def _default_width_model(self, width):
        """설정값에서 첫 프레임용 원근 차선 폭 모델을 만듭니다."""
        y_values = np.asarray(
            [cfg.TRACK_TOP_RATIO, cfg.TRACK_BOTTOM_RATIO],
            dtype=np.float64,
        )
        widths = np.asarray([
            self._default_boundaries(y_ratio, width)[1] / width
            for y_ratio in y_values
        ])
        return np.polyfit(y_values, widths, 1)

    def _offset_lane_width_px(self, width, right_item=None, left_item=None):
        """회피 시 목표를 옆 차선으로 옮길 때 쓸 차선 폭(픽셀)을 구합니다.

        학습 폭 모델만 쓰면 WIDTH_MODEL_NEAR_MIN(280px) 등으로 실제보다
        작게 잡혀 차선 변경이 끝까지 안 되는 경우가 있어, 이번 프레임에
        보이는 오른쪽/양쪽 경계 측정값과 기본 밴드 폭 중 가장 큰 값을
        사용합니다.
        """
        width_model = (
            self.width_coefficients
            if self.width_coefficients is not None
            else self._default_width_model(width)
        )
        model_width = clamp(
            float(np.polyval(
                width_model, cfg.CONTROL_Y_RATIOS[0]
            )) * width,
            scale_x(cfg.TRACK_MIN_LANE_WIDTH, width),
            min(scale_x(cfg.TRACK_MAX_LANE_WIDTH, width), width * 0.985),
        )
        candidates = [
            model_width,
            scale_x(cfg.LANE_BANDS[0]["width"], width),
        ]
        if right_item is not None:
            candidates.append(float(right_item["lane_width"]))
        if (
            right_item is not None
            and left_item is not None
            and not left_item.get("inferred_boundary", False)
        ):
            candidates.append(
                float(right_item["right"] - left_item["left"])
            )
        return max(candidates)

    def _boundary_items_at_control(self, height, width, measurements, status):
        """제어 기준 높이(CONTROL_Y_RATIOS[0])에서 가장 가까운 좌우 경계를
        고릅니다. 회피 오프셋 폭 계산과 guard 계산에서 같은 항목을 씁니다.
        """
        target_y = height * cfg.CONTROL_Y_RATIOS[0]
        maximum_y_gap = height * cfg.BOUNDARY_GUARD_MAX_Y_GAP_RATIO
        right_values = [
            item for item in measurements if item["right"] is not None
        ]
        left_values = [
            item for item in measurements if item["left"] is not None
        ]
        if status == "MEMORY":
            right_values = []
            left_values = []
        elif status == "SINGLE_LEFT" and not right_values:
            right_values = [
                {
                    **item,
                    "right": item["left"] + item["lane_width"],
                    "inferred_boundary": True,
                }
                for item in left_values
            ]
        elif status == "SINGLE_RIGHT" and not left_values:
            left_values = [
                {
                    **item,
                    "left": item["right"] - item["lane_width"],
                    "inferred_boundary": True,
                }
                for item in right_values
            ]

        right_item = None
        if right_values:
            candidate = min(
                right_values,
                key=lambda item: abs(item["y"] - target_y),
            )
            if abs(candidate["y"] - target_y) <= maximum_y_gap:
                right_item = candidate

        left_item = None
        if left_values:
            candidate = min(
                left_values,
                key=lambda item: abs(item["y"] - target_y),
            )
            if abs(candidate["y"] - target_y) <= maximum_y_gap:
                left_item = candidate
        return right_item, left_item

    def _center_line_rows(self, height):
        return np.linspace(
            height * cfg.TRACK_TOP_RATIO,
            height * cfg.TRACK_BOTTOM_RATIO,
            cfg.TRACK_ROW_COUNT,
        )

    def _collect_center_line_measurements(self, mask, height, width):
        """1·2차선 사이 점선(중앙선) 세그먼트만 행 단위로 추적합니다."""
        rows = self._center_line_rows(height)
        default_x = scale_x(cfg.CENTER_LINE_TARGET_X, width)
        max_segment_width = scale_x(cfg.CENTER_LINE_MAX_SEGMENT_WIDTH, width)
        tracked = []
        missing_rows = 0

        for y in sorted(rows, reverse=True):
            segments = [
                item for item in self._row_segments(mask, y)
                if item["width"] <= max_segment_width
            ]
            if not segments:
                missing_rows += 1
                continue

            predicted = self._previous_center(y, height, default_x)
            if tracked:
                previous = tracked[-1]
                predicted = previous["x"]
                if len(tracked) >= 2:
                    older = tracked[-2]
                    dy = previous["y"] - older["y"]
                    if abs(dy) > 1.0:
                        slope = (previous["x"] - older["x"]) / dy
                        predicted += slope * (y - previous["y"])

            margin = scale_x(
                cfg.CENTER_LINE_SEARCH_MARGIN
                + min(missing_rows, 3) * cfg.CENTER_LINE_GAP_GROWTH,
                width,
            )
            usable = [
                item for item in segments
                if abs(item["x"] - predicted) <= margin
            ]
            if not usable and not tracked:
                x_min = width * cfg.CENTER_LINE_SEARCH_MIN_RATIO
                x_max = width * cfg.CENTER_LINE_SEARCH_MAX_RATIO
                usable = [
                    item for item in segments
                    if x_min <= item["x"] <= x_max
                ]
            if not usable:
                missing_rows += 1
                continue

            selected = min(
                usable,
                key=lambda item: (
                    abs(item["x"] - predicted)
                    - min(item["strength"], 600.0) * 0.006
                ),
            )
            tracked.append({
                "y": float(y),
                "x": selected["x"],
            })
            missing_rows = 0

        lane_width = scale_x(cfg.LANE_BANDS[0]["width"], width)
        return [
            {
                "center": item["x"],
                "left": None,
                "right": None,
                "lane_width": lane_width,
                "source": "CENTER",
                "confidence": 1.0,
                "y": item["y"],
                "forward": self._forward_value(item["y"], height),
                "segments": [],
            }
            for item in sorted(tracked, key=lambda value: value["y"])
        ]

    def _fit_center_path(self, measurements):
        """중앙선 측정값만으로 경로를 맞춥니다."""
        if len(measurements) < cfg.CENTER_LINE_MIN_POINTS:
            return None, [], 0.0
        coverage = (
            max(item["forward"] for item in measurements)
            - min(item["forward"] for item in measurements)
        )
        if coverage < cfg.CENTER_LINE_MIN_COVERAGE:
            return None, [], 0.0

        forward = np.asarray(
            [item["forward"] for item in measurements], dtype=np.float64
        )
        centers = np.asarray(
            [item["center"] for item in measurements], dtype=np.float64
        )
        weights = np.asarray(
            [item["confidence"] for item in measurements], dtype=np.float64
        )
        coefficients = np.polyfit(forward, centers, 2, w=weights)
        coefficients = np.pad(coefficients, (1, 0))
        rmse = float(np.sqrt(np.mean(
            (centers - np.polyval(coefficients, forward)) ** 2
        )))
        if rmse > cfg.CENTER_LINE_MAX_FIT_RMSE:
            return None, measurements, 0.0

        point_ratio = min(1.0, len(measurements) / cfg.TRACK_ROW_COUNT)
        coverage_ratio = min(1.0, coverage / 0.65)
        confidence = point_ratio * 0.55 + coverage_ratio * 0.45
        if confidence < cfg.CENTER_LINE_MIN_CONFIDENCE:
            return None, measurements, confidence
        return coefficients, measurements, confidence

    def _path_jump_limits(self):
        if cfg.CENTER_LINE_MODE:
            return (
                cfg.CENTER_LINE_MAX_NEAR_JUMP,
                cfg.CENTER_LINE_MAX_MIDDLE_JUMP,
                cfg.CENTER_LINE_MAX_PREVIEW_JUMP,
                cfg.CENTER_LINE_MAX_FAR_JUMP,
            )
        return (
            cfg.TRACK_MAX_NEAR_JUMP,
            cfg.TRACK_MAX_MIDDLE_JUMP,
            cfg.TRACK_MAX_PREVIEW_JUMP,
            cfg.TRACK_MAX_FAR_JUMP,
        )

    @staticmethod
    def _width_tolerance(predicted_width, width):
        return max(
            scale_x(cfg.TRACK_WIDTH_PAIR_TOLERANCE, width),
            predicted_width * cfg.TRACK_WIDTH_PAIR_TOLERANCE_RATIO,
        )

    def _track_right_boundary(self, mask, rows):
        """아래에서 위로 이어지는 오른쪽 실선을 먼저 고정 추적합니다."""
        height, width = mask.shape
        minimum_x = scale_x(cfg.TRACK_RIGHT_MIN_X, width)
        tracked = []
        missing_rows = 0

        for y in sorted(rows, reverse=True):
            segments = self._row_segments(mask, y)
            y_ratio = float(y / max(1, height))
            default_center, default_width = self._default_boundaries(
                y_ratio, width
            )
            predicted_center = self._previous_center(
                y, height, default_center
            )
            if self.width_coefficients is None:
                predicted_width = default_width
            else:
                predicted_width = clamp(
                    float(np.polyval(
                        self.width_coefficients, y_ratio
                    )) * width,
                    scale_x(cfg.TRACK_MIN_LANE_WIDTH, width),
                    min(
                        scale_x(cfg.TRACK_MAX_LANE_WIDTH, width),
                        width * 0.985,
                    ),
                )
            offset_ratio = (
                cfg.TRACK_RIGHT_LOCKED_MIN_OFFSET_RATIO
                if self.path_locked
                else cfg.TRACK_RIGHT_MIN_OFFSET_RATIO
            )
            side_minimum = predicted_center + predicted_width * offset_ratio
            candidates = [
                item for item in segments
                if item["x"] >= max(minimum_x, side_minimum)
            ]
            if not candidates:
                missing_rows += 1
                continue

            predicted = predicted_center + predicted_width * 0.5
            if tracked:
                previous = tracked[-1]
                predicted = previous["right"]
                if len(tracked) >= 2:
                    older = tracked[-2]
                    dy = previous["y"] - older["y"]
                    if abs(dy) > 1.0:
                        slope = (
                            previous["right"] - older["right"]
                        ) / dy
                        predicted += slope * (y - previous["y"])

            margin = scale_x(
                cfg.TRACK_RIGHT_SEARCH_MARGIN
                + min(missing_rows, 2) * cfg.TRACK_RIGHT_GAP_GROWTH,
                width,
            )
            if not tracked and self.path_locked:
                margin = max(
                    margin,
                    predicted_width * cfg.TRACK_RIGHT_LOCKED_SEARCH_WIDTH_RATIO,
                )
            usable = [
                item for item in candidates
                if abs(item["x"] - predicted) <= margin
            ]
            if not usable:
                missing_rows += 1
                continue

            selected = min(
                usable,
                key=lambda item: (
                    abs(item["x"] - predicted)
                    - min(item["strength"], 600.0) * 0.006
                ),
            )
            tracked.append({
                "y": float(y),
                "right": selected["x"],
                "right_segment": selected,
                "segments": segments,
            })
            missing_rows = 0

        return sorted(tracked, key=lambda item: item["y"])

    def _width_model_is_valid(self, model, width):
        """원근에 맞지 않는 지나치게 좁은 차선 폭 모델을 거절합니다."""
        near_ratio = cfg.CONTROL_Y_RATIOS[0]
        far_ratio = cfg.CONTROL_Y_RATIOS[2]
        near_width = float(np.polyval(model, near_ratio)) * width
        far_width = float(np.polyval(model, far_ratio)) * width
        if not (
            scale_x(cfg.WIDTH_MODEL_NEAR_MIN, width)
            <= near_width
            <= scale_x(cfg.WIDTH_MODEL_NEAR_MAX, width)
        ):
            return False
        if not (
            scale_x(cfg.WIDTH_MODEL_FAR_MIN, width)
            <= far_width
            <= scale_x(cfg.WIDTH_MODEL_FAR_MAX, width)
        ):
            return False
        if near_width < far_width * cfg.WIDTH_MODEL_MIN_GROWTH_RATIO:
            return False

        if self.width_coefficients is not None:
            for y_ratio in cfg.CONTROL_Y_RATIOS:
                old_width = float(np.polyval(
                    self.width_coefficients, y_ratio
                )) * width
                new_width = float(np.polyval(model, y_ratio)) * width
                maximum_change = (
                    max(1.0, old_width)
                    * cfg.WIDTH_MODEL_MAX_CHANGE_RATIO
                )
                if abs(new_width - old_width) > maximum_change:
                    return False
        return True

    def _fit_width_model(self, right_track, height, width):
        """먼 곳의 깨끗한 선 쌍부터 이어지는 원근 폭만 학습합니다.

        횡단보도는 아래쪽 여러 행에서 반복되므로 단순히 지지 행이 가장 많은
        폭을 고르면 오히려 횡단보도를 차선으로 선택합니다. 먼저 화면 위쪽에서
        기본 원근 폭과 가장 가까운 후보를 잡고, 그 후보와 매끄럽게 이어지는
        폭만 아래쪽으로 확장합니다.
        """
        minimum_width = scale_x(cfg.TRACK_MIN_LANE_WIDTH, width)
        maximum_width = min(
            scale_x(cfg.TRACK_MAX_LANE_WIDTH, width),
            width * 0.985,
        )
        rows = []

        for item in right_track:
            candidates = []
            for segment in item["segments"]:
                lane_width = item["right"] - segment["x"]
                if not minimum_width <= lane_width <= maximum_width:
                    continue
                candidate = {
                    "y_ratio": item["y"] / max(1.0, float(height)),
                    "width_ratio": lane_width / max(1.0, float(width)),
                    "left": segment["x"],
                    "lane_width": lane_width,
                    "strength": segment["strength"],
                }
                candidates.append(candidate)
            if candidates:
                rows.append({
                    "y_ratio": item["y"] / max(1.0, float(height)),
                    "candidates": candidates,
                })

        default_model = self._default_width_model(width)
        selected = []
        local_model = None

        for row in rows:
            default_width = float(np.polyval(
                default_model, row["y_ratio"]
            )) * width
            if not selected:
                predicted = default_width
                tolerance = max(
                    scale_x(cfg.TRACK_WIDTH_SEED_TOLERANCE, width),
                    default_width * 0.80,
                )
            elif len(selected) == 1:
                first = selected[0]
                first_default = float(np.polyval(
                    default_model, first["y_ratio"]
                )) * width
                predicted = first["lane_width"] * (
                    default_width / max(1.0, first_default)
                )
                tolerance = max(
                    scale_x(cfg.TRACK_WIDTH_SECOND_TOLERANCE, width),
                    predicted * 0.18,
                )
            else:
                predicted = float(np.polyval(
                    local_model, row["y_ratio"]
                )) * width
                tolerance = self._width_tolerance(predicted, width)

            if not minimum_width <= predicted <= maximum_width:
                continue
            candidate = min(
                row["candidates"],
                key=lambda value: abs(value["lane_width"] - predicted),
            )
            if abs(candidate["lane_width"] - predicted) > tolerance:
                continue
            selected.append(candidate)

            if len(selected) >= 2:
                y_values = np.asarray([
                    value["y_ratio"] for value in selected
                ])
                width_values = np.asarray([
                    value["width_ratio"] for value in selected
                ])
                strengths = np.asarray([
                    min(value["strength"], 500.0)
                    for value in selected
                ])
                candidate_model = np.polyfit(
                    y_values, width_values, 1, w=strengths
                )
                if (
                    cfg.TRACK_WIDTH_MODEL_MIN_SLOPE
                    <= candidate_model[0]
                    <= cfg.TRACK_WIDTH_MODEL_MAX_SLOPE
                ):
                    local_model = candidate_model
                else:
                    selected.pop()

        reliable = len(selected) >= cfg.TRACK_WIDTH_MIN_PAIR_ROWS
        if reliable:
            coverage = (
                max(value["y_ratio"] for value in selected)
                - min(value["y_ratio"] for value in selected)
            )
            reliable = coverage >= cfg.TRACK_WIDTH_MIN_PAIR_COVERAGE
        # (2026-08-04) 예전엔 직전 프레임이 단일 차선 모드였으면 폭을
        # 무조건 동결했습니다. S자처럼 한쪽 선이 길게 안 보이는 구간
        # 전체에서 진입 직전 폭이 그대로 얼어붙어, 그 폭이 조금만 좁아도
        # 역산된 중앙이 계속 오른쪽으로 쏠리는 문제가 실차에서 확인됐습니다.
        # 아래 짝지어진 행 수/커버리지 검사와 _width_model_is_valid,
        # 그리고 완만한 블렌딩(TRACK_WIDTH_FILTER)이 이미 노이즈를 걸러내므로
        # 단일차선 모드 중에도 우연히 양쪽이 잠깐 보이면 계속 다듬게 둡니다.
        if reliable:
            y_values = np.asarray([
                value["y_ratio"] for value in selected
            ])
            width_values = np.asarray([
                value["width_ratio"] for value in selected
            ])
            strengths = np.asarray([
                min(value["strength"], 500.0)
                for value in selected
            ])
            model = np.polyfit(y_values, width_values, 1, w=strengths)
            reliable = self._width_model_is_valid(model, width)
            if reliable:
                if self.width_coefficients is None:
                    self.width_coefficients = model
                else:
                    alpha = cfg.TRACK_WIDTH_FILTER
                    self.width_coefficients = (
                        alpha * self.width_coefficients
                        + (1.0 - alpha) * model
                    )
                self.width_model_age = 0

        if not reliable:
            self.width_model_age = min(
                self.width_model_age + 1,
                cfg.SINGLE_LANE_WIDTH_MAX_AGE + 1,
            )

        model = (
            self.width_coefficients
            if self.width_coefficients is not None
            else default_model
        )
        return model, reliable

    def _collect_measurements(self, mask):
        """오른쪽 실선과 원근 차선 폭으로 각 높이의 중앙점을 복원합니다."""
        height, width = mask.shape
        rows = np.linspace(
            height * cfg.TRACK_TOP_RATIO,
            height * cfg.TRACK_BOTTOM_RATIO,
            cfg.TRACK_ROW_COUNT,
        )
        right_track = self._track_right_boundary(mask, rows)
        width_model, width_reliable = self._fit_width_model(
            right_track, height, width
        )
        minimum_width = scale_x(cfg.TRACK_MIN_LANE_WIDTH, width)
        maximum_width = min(
            scale_x(cfg.TRACK_MAX_LANE_WIDTH, width),
            width * 0.985,
        )
        measurements = []

        for item in right_track:
            y_ratio = item["y"] / max(1.0, float(height))
            predicted_width = clamp(
                float(np.polyval(width_model, y_ratio)) * width,
                minimum_width,
                maximum_width,
            )
            left_candidates = []
            for segment in item["segments"]:
                measured_width = item["right"] - segment["x"]
                if minimum_width <= measured_width <= maximum_width:
                    left_candidates.append((segment, measured_width))

            selected_left = None
            if left_candidates:
                selected_left = min(
                    left_candidates,
                    key=lambda value: abs(value[1] - predicted_width),
                )
                tolerance = self._width_tolerance(predicted_width, width)
                if abs(selected_left[1] - predicted_width) > tolerance * 1.20:
                    selected_left = None

            if selected_left is not None:
                left_segment, measured_lane_width = selected_left
                left_x = left_segment["x"]
                center_width = (
                    predicted_width
                    + (measured_lane_width - predicted_width)
                    * cfg.TRACK_PAIR_CENTER_BLEND
                )
                center = item["right"] - center_width * 0.5
                lane_width = measured_lane_width
                source = "BOTH"
                confidence = 1.0
            else:
                lane_width = predicted_width
                left_x = None
                center = item["right"] - lane_width * 0.5
                source = "RIGHT"
                confidence = 0.68 if width_reliable else 0.56

            measurements.append({
                "center": center,
                "left": left_x,
                "right": item["right"],
                "lane_width": lane_width,
                "source": source,
                "confidence": confidence,
                "y": item["y"],
                "forward": self._forward_value(item["y"], height),
                "segments": item["segments"],
            })
        return measurements

    def _collect_single_lane_measurements(self, mask, side):
        """보이는 한쪽 경계와 학습된(또는 기본) 차선 폭으로 중앙점을 복원합니다."""
        height, width = mask.shape
        width_model = (
            self.width_coefficients
            if self.width_coefficients is not None
            else self._default_width_model(width)
        )
        minimum_width = scale_x(cfg.TRACK_MIN_LANE_WIDTH, width)
        maximum_width = min(
            scale_x(cfg.TRACK_MAX_LANE_WIDTH, width),
            width * 0.985,
        )
        rows = np.linspace(
            height * cfg.TRACK_TOP_RATIO,
            height * cfg.TRACK_BOTTOM_RATIO,
            cfg.TRACK_ROW_COUNT,
        )
        direction = 1.0 if side == "RIGHT" else -1.0
        measurements = []

        for y in rows:
            y_ratio = float(y / max(1.0, float(height)))
            lane_width = clamp(
                float(np.polyval(
                    width_model, y_ratio
                )) * width,
                minimum_width,
                maximum_width,
            )
            predicted_center = self._previous_center(
                y, height, width * 0.5
            )
            predicted_boundary = (
                predicted_center + direction * lane_width * 0.5
            )
            side_limit = (
                predicted_center
                + direction
                * lane_width
                * cfg.SINGLE_LANE_SIDE_OFFSET_RATIO
            )
            search_margin = min(
                scale_x(cfg.SINGLE_LANE_SEARCH_MARGIN, width),
                lane_width * 0.32,
            )

            candidates = []
            for segment in self._row_segments(mask, y):
                if side == "RIGHT" and segment["x"] < side_limit:
                    continue
                if side == "LEFT" and segment["x"] > side_limit:
                    continue
                distance = abs(segment["x"] - predicted_boundary)
                if distance <= search_margin:
                    candidates.append((segment, distance))
            if not candidates:
                continue

            segment, distance = min(
                candidates,
                key=lambda value: (
                    value[1]
                    - min(value[0]["strength"], 600.0) * 0.004
                ),
            )
            center = segment["x"] - direction * lane_width * 0.5
            confidence = 0.72 + 0.18 * (
                1.0 - distance / max(1.0, search_margin)
            )
            measurements.append({
                "center": center,
                "left": segment["x"] if side == "LEFT" else None,
                "right": segment["x"] if side == "RIGHT" else None,
                "lane_width": lane_width,
                "source": side,
                "confidence": confidence,
                "y": float(y),
                "forward": self._forward_value(y, height),
                "segments": [segment],
            })
        return measurements

    def _fit_single_lane_path(self, measurements, height, width):
        """단일 경계 경로를 엄격히 검사하고 완만한 2차 곡선으로 맞춥니다."""
        if len(measurements) < cfg.SINGLE_LANE_MIN_POINTS:
            return None, measurements, 0.0

        def coverage_of(items):
            return (
                max(item["forward"] for item in items)
                - min(item["forward"] for item in items)
            )

        if coverage_of(measurements) < cfg.SINGLE_LANE_MIN_COVERAGE:
            return None, measurements, 0.0

        def fit(items):
            forward = np.asarray([
                item["forward"] for item in items
            ], dtype=np.float64)
            centers = np.asarray([
                item["center"] for item in items
            ], dtype=np.float64)
            weights = np.asarray([
                item["confidence"] for item in items
            ], dtype=np.float64)
            coefficients = np.polyfit(
                forward, centers, 2, w=weights
            )
            coefficients = np.pad(coefficients, (1, 0))
            residuals = np.abs(
                centers - np.polyval(coefficients, forward)
            )
            rmse = float(np.sqrt(np.mean(residuals ** 2)))
            return coefficients, residuals, rmse

        active = list(measurements)
        for _ in range(2):
            coefficients, residuals, rmse = fit(active)
            keep = residuals <= cfg.SINGLE_LANE_RESIDUAL_LIMIT
            if bool(np.all(keep)):
                break
            if int(np.count_nonzero(keep)) < cfg.SINGLE_LANE_MIN_POINTS:
                return None, active, 0.0
            active = [item for item, ok in zip(active, keep) if ok]

        coefficients, _, rmse = fit(active)
        active_coverage = coverage_of(active)
        if (
            active_coverage < cfg.SINGLE_LANE_MIN_COVERAGE
            or rmse > cfg.SINGLE_LANE_MAX_RMSE
        ):
            return None, active, 0.0

        # 직전 경로가 있을 때만(콜드 스타트가 아닐 때만) 프레임 간 이동을
        # 제한합니다 — 비교할 직전 경로가 없으면 제한할 대상이 없습니다.
        if self.path_coefficients is not None:
            maximum_jumps = (
                cfg.SINGLE_LANE_MAX_NEAR_JUMP,
                cfg.SINGLE_LANE_MAX_MIDDLE_JUMP,
                cfg.SINGLE_LANE_MAX_FAR_JUMP,
            )
            for y_ratio, maximum_jump in zip(
                cfg.CONTROL_Y_RATIOS, maximum_jumps
            ):
                forward = self._forward_value(height * y_ratio, height)
                old_x = float(np.polyval(self.path_coefficients, forward))
                new_x = float(np.polyval(coefficients, forward))
                if abs(new_x - old_x) > scale_x(maximum_jump, width):
                    return None, active, 0.0

        point_ratio = min(
            1.0, len(active) / max(1.0, cfg.TRACK_ROW_COUNT * 0.65)
        )
        coverage_ratio = min(1.0, active_coverage / 0.58)
        rmse_ratio = max(
            0.0, 1.0 - rmse / max(1.0, cfg.SINGLE_LANE_MAX_RMSE)
        )
        confidence = (
            0.55
            + point_ratio * 0.20
            + coverage_ratio * 0.15
            + rmse_ratio * 0.10
        )
        return coefficients, active, min(0.95, confidence)

    def _try_single_lane(self, mask, height, width):
        """한쪽 경계만 보일 때 단일 경계 추적을 시도합니다.

        직전에 양쪽 차선을 락온한 상태라면 그 경로/학습된 폭을 그대로
        쓰고, 그렇지 않으면(LANE:LOST 직후 등 "직전 경로"가 아예 없는
        상태) 설정값 기반 기본 폭 모델로 차갑게 시작합니다(2026-08-04
        추가) — 벽 때문에 한쪽만 계속 안 보이는 구간에서 양쪽을 동시에
        다시 잡을 때까지 완전히 멈춰 서 있던 문제를 막기 위함입니다.
        콜드 스타트 결과도 _is_plausible_path 신뢰성 검사와 프레임별
        이동 제한을 그대로 통과해야 실제 조향에 반영됩니다.
        """
        if not cfg.SINGLE_LANE_ENABLED:
            return None, [], 0.0, None
        if self.single_lane_frames >= cfg.SINGLE_LANE_MAX_FRAMES:
            return None, [], 0.0, None
        # width_model_age는 "학습된 폭이 너무 오래돼 못 미덥다"는 뜻이라,
        # 학습된 폭이 아예 없어 기본값으로 대체하는 콜드 스타트에는
        # 적용하지 않습니다.
        if (
            self.width_coefficients is not None
            and self.width_model_age > cfg.SINGLE_LANE_WIDTH_MAX_AGE
        ):
            return None, [], 0.0, None

        sides = ["RIGHT", "LEFT"]
        if self.single_lane_side in sides:
            sides.remove(self.single_lane_side)
            sides.insert(0, self.single_lane_side)

        candidates = []
        for side in sides:
            measurements = self._collect_single_lane_measurements(
                mask, side
            )
            coefficients, inliers, confidence = (
                self._fit_single_lane_path(
                    measurements, height, width
                )
            )
            if coefficients is None:
                continue
            preference = 0.03 if side == self.single_lane_side else 0.0
            candidates.append((
                confidence + preference,
                coefficients,
                inliers,
                confidence,
                side,
            ))

        if not candidates:
            return None, [], 0.0, None
        _, coefficients, inliers, confidence, side = max(
            candidates, key=lambda value: value[0]
        )
        return coefficients, inliers, confidence, side

    @staticmethod
    def _fit_path(measurements):
        """이상치를 제거하고 2차·3차 중 실제 데이터에 맞는 경로를 선택합니다."""
        if len(measurements) < cfg.TRACK_MIN_POINTS:
            return None, [], 0.0
        coverage = (
            max(item["forward"] for item in measurements)
            - min(item["forward"] for item in measurements)
        )
        if coverage < cfg.TRACK_MIN_COVERAGE:
            return None, [], 0.0

        def fit_best(items):
            forward = np.asarray(
                [item["forward"] for item in items], dtype=np.float64
            )
            centers = np.asarray(
                [item["center"] for item in items], dtype=np.float64
            )
            weights = np.asarray(
                [item["confidence"] for item in items], dtype=np.float64
            )

            quadratic = np.polyfit(
                forward, centers, 2, w=weights
            )
            quadratic = np.pad(quadratic, (1, 0))
            quadratic_rmse = float(np.sqrt(np.mean(
                (centers - np.polyval(quadratic, forward)) ** 2
            )))
            selected = quadratic
            selected_rmse = quadratic_rmse

            # 점선이 끊긴 오른쪽 단일 경계만으로 3차식을 만들면 먼 경로가
            # 화면 반대편으로 외삽될 수 있습니다. 실제 양쪽 선이 충분히
            # 넓게 확인된 경우에만 S자 3차 경로를 허용합니다.
            both_items = [
                item for item in items if item["source"] == "BOTH"
            ]
            both_coverage = (
                max(item["forward"] for item in both_items)
                - min(item["forward"] for item in both_items)
                if len(both_items) >= 2
                else 0.0
            )
            if (
                len(items) >= 7
                and len(both_items) >= cfg.TRACK_CUBIC_MIN_BOTH_POINTS
                and both_coverage >= cfg.TRACK_CUBIC_MIN_BOTH_COVERAGE
            ):
                cubic = np.polyfit(
                    forward, centers, 3, w=weights
                )
                cubic_rmse = float(np.sqrt(np.mean(
                    (centers - np.polyval(cubic, forward)) ** 2
                )))
                if cubic_rmse < quadratic_rmse * cfg.TRACK_CUBIC_RMSE_RATIO:
                    selected = cubic
                    selected_rmse = cubic_rmse
            return selected, selected_rmse, forward, centers

        active = list(measurements)
        for _ in range(2):
            coefficients, rmse, forward, centers = fit_best(active)
            residuals = np.abs(
                centers - np.polyval(coefficients, forward)
            )
            adaptive_limit = max(
                cfg.TRACK_RESIDUAL_LIMIT,
                float(np.median(residuals)) * 2.8,
            )
            keep = residuals <= adaptive_limit
            if bool(np.all(keep)):
                break
            if int(np.count_nonzero(keep)) < cfg.TRACK_MIN_POINTS:
                return None, active, 0.0
            active = [item for item, ok in zip(active, keep) if ok]

        coefficients, rmse, _, _ = fit_best(active)
        if rmse > cfg.TRACK_MAX_FIT_RMSE:
            return None, active, 0.0
        active_coverage = (
            max(item["forward"] for item in active)
            - min(item["forward"] for item in active)
        )
        both_ratio = sum(
            item["source"] == "BOTH" for item in active
        ) / len(active)
        point_ratio = min(1.0, len(active) / cfg.TRACK_ROW_COUNT)
        coverage_ratio = min(1.0, active_coverage / 0.70)
        confidence = (
            point_ratio * 0.45
            + both_ratio * 0.35
            + coverage_ratio * 0.20
        )
        if confidence < cfg.TRACK_MIN_CONFIDENCE:
            return None, active, confidence
        return coefficients, active, confidence

    def _is_plausible_path(self, height, width):
        """방금 반영한 경로의 헤딩/곡률이 실제 코스에서 나올 수 있는
        범위인지 확인합니다. 커튼 주름·주차칸 격자선 등 차선이 아닌
        흰색 물체를 잘못 추적했을 때 걸러내기 위한 검사입니다.
        """
        path_x = np.asarray([
            self._path_x(y_ratio, height)
            for y_ratio in cfg.CONTROL_Y_RATIOS
        ])
        heading_error = (
            (path_x[2] - path_x[0]) * cfg.REFERENCE_WIDTH / width
        )
        signed_curvature = (
            (path_x[2] - 2.0 * path_x[1] + path_x[0])
            * cfg.REFERENCE_WIDTH / width
        )
        return (
            abs(heading_error) <= cfg.MAX_PLAUSIBLE_HEADING_ERROR
            and abs(signed_curvature) <= cfg.MAX_PLAUSIBLE_CURVATURE
        )

    def _accept_and_filter_path(self, coefficients, height, width):
        """가까운·중간·먼 경로가 한 프레임에 반전되지 않게 제한합니다."""
        if coefficients is None:
            return False
        previous_coefficients = self.path_coefficients
        if self.path_coefficients is not None:
            sample_ratios = (
                cfg.CONTROL_Y_RATIOS[0],
                cfg.CONTROL_Y_RATIOS[1],
                cfg.PREVIEW_Y_RATIO,
                cfg.CONTROL_Y_RATIOS[2],
            )
            maximum_jumps = self._path_jump_limits()
            forwards = np.asarray([
                self._forward_value(height * ratio, height)
                for ratio in sample_ratios
            ], dtype=np.float64)
            old_values = np.polyval(self.path_coefficients, forwards)
            new_values = np.polyval(coefficients, forwards)
            bounded_values = np.asarray([
                old + clamp(
                    new - old,
                    -scale_x(limit, width),
                    scale_x(limit, width),
                )
                for old, new, limit in zip(
                    old_values, new_values, maximum_jumps
                )
            ], dtype=np.float64)
            unique_forwards, unique_indices = np.unique(
                forwards, return_index=True
            )
            unique_values = bounded_values[unique_indices]
            degree = min(3, len(unique_forwards) - 1)
            fitted = np.polyfit(unique_forwards, unique_values, degree)
            coefficients = np.pad(fitted, (4 - len(fitted), 0))

            alpha = cfg.TRACK_PATH_FILTER
            self.path_coefficients = (
                alpha * self.path_coefficients
                + (1.0 - alpha) * coefficients
            )
        else:
            self.path_coefficients = coefficients

        # previous_coefficients가 없으면(첫 락온 시도) 이 검사를 건너뜁니다.
        # 지킬 "직전 경로"가 없는 상태에서 첫 프레임마저 거부하면 되돌아갈
        # 곳이 없어 영원히 락온에 실패합니다(실차에서 실제로 재현됨: 출발
        # 지점의 첫 경로가 임계값에 살짝 걸려 30초 넘게 전혀 출발하지 못함).
        if previous_coefficients is None:
            return True

        if not self._is_plausible_path(height, width):
            # 이번 프레임의 경로는 신뢰하지 않고 직전 경로를 그대로
            # 유지합니다(감지 실패와 동일하게 처리 → update()에서 자연히
            # MEMORY로 폴백).
            self.path_coefficients = previous_coefficients
            return False
        return True

    def _path_x(self, y_ratio, height):
        y = height * y_ratio
        forward = self._forward_value(y, height)
        return float(np.polyval(self.path_coefficients, forward))

    def _calculate_control(self, height, width, measurements, status):
        """가까운 경계 안전거리와 전방 preview 경로로 조향값을 구합니다."""
        path_x = np.asarray([
            self._path_x(y_ratio, height)
            for y_ratio in cfg.CONTROL_Y_RATIOS
        ])
        heading_error = float(
            (path_x[2] - path_x[0]) * cfg.REFERENCE_WIDTH / width
        )
        signed_curvature = float(
            (path_x[2] - 2.0 * path_x[1] + path_x[0])
            * cfg.REFERENCE_WIDTH / width
        )
        speed_span = max(1.0, cfg.BASE_SPEED - cfg.CURVE_SPEED)
        speed_ratio = clamp(
            (self.previous_speed - cfg.CURVE_SPEED) / speed_span,
            0.0,
            1.0,
        )
        if status in ("TRACKING", "CENTER_TRACKING"):
            requested_curve_margin = clamp(
                (
                    heading_error * cfg.CURVE_OUTSIDE_HEADING_GAIN
                    + signed_curvature * cfg.CURVE_OUTSIDE_CURVATURE_GAIN
                ) * (1.0 + speed_ratio * cfg.CURVE_OUTSIDE_SPEED_GAIN),
                -cfg.CURVE_OUTSIDE_MAX,
                cfg.CURVE_OUTSIDE_MAX,
            )
        else:
            requested_curve_margin = 0.0
        self.curve_target_offset_state = (
            cfg.CURVE_OUTSIDE_FILTER * self.curve_target_offset_state
            + (1.0 - cfg.CURVE_OUTSIDE_FILTER)
            * requested_curve_margin
        )
        curve_margin = self.curve_target_offset_state
        base_target_x = (
            cfg.CENTER_LINE_TARGET_X
            if cfg.CENTER_LINE_MODE
            else cfg.CONTROL_TARGET_X[0]
        )
        near_target = scale_x(base_target_x + curve_margin, width)
        if cfg.CENTER_LINE_MODE:
            right_item, left_item = None, None
        else:
            right_item, left_item = self._boundary_items_at_control(
                height, width, measurements, status
            )
        offset_lane_width_px = 0.0
        # 오프셋을 적용하기 전, "평소 정면 기준선"을 따로 기억해둡니다.
        # 회피 중 왼쪽 점선이 이 기준선 왼쪽/오른쪽 중 어디 있는지로
        # "점선을 넘었는지"를 판정하는 데 씁니다(오프셋이 걸린 near_target
        # 이 아니라 이 고정 기준으로 비교해야 오프셋 크기와 무관하게
        # 일관된 판정이 됩니다).
        vehicle_center_x = near_target
        if self._lane_offset_lanes != 0.0:
            # 장애물 회피 중: 목표 위치를 차선 폭 한 칸만큼 옆으로 옮깁니다.
            # 폭은 학습 모델·실측 경계·기본 밴드 중 가장 큰 값을 씁니다.
            offset_lane_width_px = self._offset_lane_width_px(
                width, right_item, left_item
            )
            near_target += (
                cfg.AVOID_LANE_DIRECTION
                * self._lane_offset_lanes
                * offset_lane_width_px
            )
        near_error = float(
            (path_x[0] - near_target) * cfg.REFERENCE_WIDTH / width
        )
        preview_x = self._path_x(cfg.PREVIEW_Y_RATIO, height)
        preview_error = float(clamp(
            (preview_x - near_target) * cfg.REFERENCE_WIDTH / width,
            -cfg.PREVIEW_ERROR_LIMIT,
            cfg.PREVIEW_ERROR_LIMIT,
        ))
        far_direction = float(
            (path_x[2] - preview_x) * cfg.REFERENCE_WIDTH / width
        )

        if status in ("TRACKING", "CENTER_TRACKING"):
            preview_weight = min(
                cfg.PREVIEW_MAX_WEIGHT,
                cfg.PREVIEW_TRACKING_WEIGHT
                + speed_ratio * cfg.PREVIEW_SPEED_GAIN,
            )
        elif status in ("SINGLE_LEFT", "SINGLE_RIGHT"):
            preview_weight = 0.0
            if len(measurements) >= cfg.PREVIEW_SINGLE_MIN_POINTS:
                forward_values = [
                    item["forward"] for item in measurements
                ]
                coverage = max(forward_values) - min(forward_values)
                preview_forward = self._forward_value(
                    height * cfg.PREVIEW_Y_RATIO, height
                )
                preview_observed = (
                    min(forward_values) - 0.03
                    <= preview_forward
                    <= max(forward_values) + 0.03
                )
                if (
                    coverage >= cfg.PREVIEW_SINGLE_MIN_COVERAGE
                    and preview_observed
                ):
                    ramp = min(
                        1.0,
                        self.single_lane_frames
                        / max(1.0, cfg.PREVIEW_SINGLE_RAMP_FRAMES),
                    )
                    preview_weight = (
                        cfg.PREVIEW_SINGLE_WEIGHT * ramp
                    )
        else:
            # MEMORY에서는 오래된 먼 경로로 계속 꺾지 않습니다.
            preview_weight = 0.0

        near_right = None
        right_guard = 0.0
        right_hard = False
        if not cfg.CENTER_LINE_MODE and right_item is not None:
            right_inferred = bool(
                right_item.get("inferred_boundary", False)
            )
            near_right = right_item["right"]
            right_clearance = near_right - near_target
            right_soft = max(
                right_item["lane_width"]
                * cfg.RIGHT_SAFE_CLEARANCE_RATIO,
                scale_x(cfg.RIGHT_SAFE_CLEARANCE_MIN, width),
            )
            right_hard_limit = max(
                right_item["lane_width"]
                * cfg.BOUNDARY_HARD_CLEARANCE_RATIO,
                scale_x(cfg.BOUNDARY_HARD_CLEARANCE_MIN, width),
            )
            right_shortage = max(
                0.0,
                right_soft - right_clearance,
            ) * cfg.REFERENCE_WIDTH / width
            right_guard = clamp(
                (right_shortage - cfg.RIGHT_GUARD_DEADBAND)
                * cfg.RIGHT_GUARD_GAIN,
                0.0,
                cfg.RIGHT_GUARD_MAX,
            )
            if right_inferred:
                right_guard *= cfg.INFERRED_BOUNDARY_GUARD_SCALE
            right_hard = (
                not right_inferred
                and right_clearance <= right_hard_limit
            )
            if right_guard > 0.0 and preview_error > near_error:
                # [2026-08-06] 하한(BOUNDARY_PREVIEW_MIN_SCALE)을 둔다.
                # 아래 왼쪽 경계 쪽 같은 수정과 짝. 이유는 그쪽 주석 참고.
                boundary_scale = clamp(
                    (right_clearance - right_hard_limit)
                    / max(1.0, right_soft - right_hard_limit),
                    cfg.BOUNDARY_PREVIEW_MIN_SCALE,
                    1.0,
                )
                preview_weight *= boundary_scale

        left_guard = 0.0
        left_hard = False
        if not cfg.CENTER_LINE_MODE and left_item is not None:
            left_inferred = bool(
                left_item.get("inferred_boundary", False)
            )
            left_clearance = near_target - left_item["left"]
            left_soft = max(
                left_item["lane_width"]
                * cfg.RIGHT_SAFE_CLEARANCE_RATIO,
                scale_x(cfg.RIGHT_SAFE_CLEARANCE_MIN, width),
            )
            left_hard_limit = max(
                left_item["lane_width"]
                * cfg.BOUNDARY_HARD_CLEARANCE_RATIO,
                scale_x(cfg.BOUNDARY_HARD_CLEARANCE_MIN, width),
            )
            left_shortage = max(
                0.0,
                left_soft - left_clearance,
            ) * cfg.REFERENCE_WIDTH / width
            left_guard = clamp(
                (left_shortage - cfg.RIGHT_GUARD_DEADBAND)
                * cfg.RIGHT_GUARD_GAIN,
                0.0,
                cfg.RIGHT_GUARD_MAX,
            )
            if left_inferred:
                left_guard *= cfg.INFERRED_BOUNDARY_GUARD_SCALE
            left_hard = (
                not left_inferred
                and left_clearance <= left_hard_limit
            )
            if left_guard > 0.0 and preview_error < near_error:
                # [2026-08-06] 하한 0.0 -> BOUNDARY_PREVIEW_MIN_SCALE.
                # 실차 영상(20260806-0425-49, 71~81초)에서 속도가 105->165로
                # 오르자 진폭이 88->149->192로 자라는 "발산하는" 진동을
                # 확인했고, 그 원인이 여기였다. hard_boundary가 켜지면
                # clearance <= hard_limit이라 boundary_scale이 정확히 0이
                # 되고 preview_weight가 통째로 0이 된다. 그 구간 11개 표본
                # 중 8개에서 PW=0.00이었다(73%).
                # preview가 0이면 제어기는 범퍼 바로 앞 한 점
                # (CONTROL_Y_RATIOS[0]=0.81)만 보고 조향하게 되는데, 이건
                # 순수추종에서 전방주시거리를 0으로 만든 것과 같아 속도가
                # 붙으면 반드시 발진한다. 그래서 속도가 오를수록 진폭이
                # 커졌다.
                # t=80.0 프레임이 메커니즘 그대로다: PERR=-110인데 ERR=-150,
                # 즉 전방은 펴지고 있다고 알려주는데 H=1이라 PW=0.00으로
                # 그 정보를 버리고 STR=200(거의 풀 좌회전) -> 1초 뒤
                # ERR=+42로 반대쪽 경계를 넘어감(LG=45, H=-1).
                # 원 의도(경계 근처에서 preview가 차를 경계 쪽으로 더
                # 끌지 않게)는 BOUNDARY_HARD_STEER_OFFSET=12가 최소 회전량을
                # 이미 강제하므로 그쪽에서 보장된다. 여기서 preview까지
                # 죽이는 건 중복이면서 발진을 만든다.
                boundary_scale = clamp(
                    (left_clearance - left_hard_limit)
                    / max(1.0, left_soft - left_hard_limit),
                    cfg.BOUNDARY_PREVIEW_MIN_SCALE,
                    1.0,
                )
                preview_weight *= boundary_scale

        if right_item is not None:
            safety_lane_width = right_item["lane_width"]
        elif left_item is not None:
            safety_lane_width = left_item["lane_width"]
        elif self.width_coefficients is not None:
            safety_lane_width = float(np.polyval(
                self.width_coefficients,
                cfg.CONTROL_Y_RATIOS[0],
            )) * width
        else:
            safety_lane_width = self._default_boundaries(
                cfg.CONTROL_Y_RATIOS[0], width
            )[1]
        lane_half_width = max(1.0, safety_lane_width * 0.5)
        safety_start = (
            lane_half_width * 2.0 * cfg.LATERAL_SAFETY_START_RATIO
        )
        safety_excess = max(0.0, abs(near_error) - safety_start)
        lateral_safety = clamp(
            safety_excess * cfg.LATERAL_SAFETY_GAIN,
            0.0,
            cfg.LATERAL_SAFETY_MAX,
        )
        if near_error < 0.0:
            lateral_safety = -lateral_safety

        # 좌우 커브 모두에서 왼쪽 점선에서 복원한 차로 중앙을 함께 사용합니다.
        # 우회전 중에는 왼쪽 점선이 짧게 비었다가 다시 보이는 경우가 많은데,
        # 오른쪽 실선+학습 폭만으로 중심을 추정하면 그 사이 오차가 누적되어
        # 오른쪽으로 쏠릴 수 있으므로, 실제로 보이는 왼쪽 점선을 직접 기준으로
        # 함께 사용해 방향과 무관하게 중심을 보정합니다. 점선이 보이는
        # 프레임만 갱신하고, 공백에서는 짧게 유지해 점선 간격마다 조향이
        # 좌우로 바뀌지 않게 합니다.
        left_anchor_error = 0.0
        left_anchor_active = False
        if (
            not cfg.CENTER_LINE_MODE
            and abs(signed_curvature) >= cfg.LEFT_ANCHOR_CURVE_TRIGGER
        ):
            if (
                left_item is not None
                and not left_item.get("inferred_boundary", False)
            ):
                anchor_center = (
                    left_item["left"] + left_item["lane_width"] * 0.5
                )
                observed_anchor_error = clamp(
                    (anchor_center - near_target)
                    * cfg.REFERENCE_WIDTH / width,
                    -cfg.LEFT_ANCHOR_ERROR_LIMIT,
                    cfg.LEFT_ANCHOR_ERROR_LIMIT,
                )
                self.left_anchor_error_state = (
                    cfg.LEFT_ANCHOR_FILTER * self.left_anchor_error_state
                    + (1.0 - cfg.LEFT_ANCHOR_FILTER)
                    * observed_anchor_error
                )
                self.left_anchor_age = 0
            else:
                self.left_anchor_age += 1
            if self.left_anchor_age <= cfg.LEFT_ANCHOR_MEMORY_FRAMES:
                left_anchor_error = self.left_anchor_error_state
                left_anchor_active = True
        else:
            self.left_anchor_age = min(
                self.left_anchor_age + 1,
                cfg.LEFT_ANCHOR_MEMORY_FRAMES + 1,
            )
            self.left_anchor_error_state *= cfg.LEFT_ANCHOR_FILTER

        raw_error = float(clamp(
            near_error
            + preview_weight * (preview_error - near_error)
            + far_direction * cfg.HEADING_CONTROL_GAIN
            + signed_curvature * cfg.SIGNED_CURVATURE_GAIN
            + lateral_safety,
            -cfg.CONTROL_ERROR_LIMIT,
            cfg.CONTROL_ERROR_LIMIT,
        ))
        if left_anchor_active:
            raw_error = float(clamp(
                (1.0 - cfg.LEFT_ANCHOR_BLEND) * raw_error
                + cfg.LEFT_ANCHOR_BLEND * left_anchor_error,
                -cfg.CONTROL_ERROR_LIMIT,
                cfg.CONTROL_ERROR_LIMIT,
            ))
        corridor_error = left_guard - right_guard
        requested_corridor_steer = clamp(
            corridor_error * cfg.BOUNDARY_STEER_GAIN,
            -cfg.BOUNDARY_STEER_MAX,
            cfg.BOUNDARY_STEER_MAX,
        )
        # 점선 공백 때문에 좌우 guard가 번갈아 잡혀도 조향 보정이 즉시
        # 반전되지 않도록 필터링합니다.
        self.corridor_steer_state = (
            cfg.BOUNDARY_STEER_FILTER * self.corridor_steer_state
            + (1.0 - cfg.BOUNDARY_STEER_FILTER)
            * requested_corridor_steer
        )
        corridor_steer = self.corridor_steer_state
        # 한 프레임의 흰 선 노이즈로 강제 조향이 걸리지 않도록 실제 경계
        # 위험이 연속해서 보일 때만 hard guard를 사용합니다.
        self.right_hard_frames = (
            self.right_hard_frames + 1 if right_hard else 0
        )
        self.left_hard_frames = (
            self.left_hard_frames + 1 if left_hard else 0
        )
        right_hard = (
            self.right_hard_frames >= cfg.BOUNDARY_HARD_CONFIRM_FRAMES
        )
        left_hard = (
            self.left_hard_frames >= cfg.BOUNDARY_HARD_CONFIRM_FRAMES
        )
        hard_boundary = (
            1 if right_hard and not left_hard
            else -1 if left_hard and not right_hard
            else 0
        )
        curvature = abs(signed_curvature)
        control_errors = np.asarray([
            near_error,
            preview_error,
            signed_curvature,
        ])
        # [2026-08-06 Claude 추가] 회피 중 "점선(왼쪽 경계)을 넘었는지"를
        # 판정하기 위해, 지금 보이는 왼쪽 경계가 평소 정면 기준선의 왼쪽/
        # 오른쪽 중 어디 있는지를 남깁니다. left_item이 이번 프레임에 안
        # 보이거나, 실제로 관측된 선이 아니라 "오른쪽 선 - 학습 폭"으로
        # 추정만 한 값(inferred_boundary)이면 None(판정 보류)입니다 —
        # 추정값은 정의상 항상 오른쪽 기준보다 왼쪽에 나오게 계산되므로
        # (right - width), 실제로 선을 넘었는지와 무관하게 항상 LEFT로
        # 잘못 나올 수 있습니다.
        if left_item is not None and not left_item.get(
            "inferred_boundary", False
        ):
            left_boundary_side = (
                "LEFT" if left_item["left"] < vehicle_center_x else "RIGHT"
            )
        else:
            left_boundary_side = None
        return (
            raw_error,
            heading_error,
            curvature,
            path_x,
            control_errors,
            right_guard,
            left_guard,
            curve_margin,
            near_right,
            lateral_safety,
            preview_error,
            preview_x,
            preview_weight,
            corridor_steer,
            hard_boundary,
            left_anchor_error,
            left_boundary_side,
            offset_lane_width_px,
        )

    def update(self, frame):
        """영상 한 장을 처리하고 목표 속도와 조향값을 반환합니다."""
        now = time.monotonic()
        if self.last_update_time is None:
            delta_time = 1.0 / 20.0
        else:
            delta_time = clamp(now - self.last_update_time, 0.01, 0.12)
        self.last_update_time = now
        offset_lane_width_px = 0.0

        roi, mask, threshold = self._make_mask(frame)
        height, width = mask.shape
        single_side = None
        if cfg.CENTER_LINE_MODE:
            measurements = self._collect_center_line_measurements(
                mask, height, width
            )
            coefficients, inliers, confidence = self._fit_center_path(
                measurements
            )
            measurements = inliers
        else:
            measurements = self._collect_measurements(mask)
            coefficients, inliers, confidence = self._fit_path(measurements)
            paired_points = sum(
                item["source"] == "BOTH" for item in inliers
            )
            # 한쪽 선만 보이는 결과를 일반 양쪽 차선 결과로 통과시키지 않습니다.
            # 잠금 이후의 단일 차선 전용 검사에서만 안전하게 허용합니다.
            if (
                coefficients is not None
                and paired_points < cfg.TRACK_WIDTH_MIN_PAIR_ROWS
                and not (
                    self.width_coefficients is not None
                    and self.width_model_age == 0
                )
            ):
                coefficients = None

            if coefficients is None:
                (
                    single_coefficients,
                    single_inliers,
                    single_confidence,
                    single_side,
                ) = self._try_single_lane(mask, height, width)
                if single_coefficients is not None:
                    coefficients = single_coefficients
                    measurements = single_inliers
                    inliers = single_inliers
                    confidence = single_confidence

        detected = self._accept_and_filter_path(
            coefficients, height, width
        )

        if detected:
            self.lost_count = 0
            self.valid_streak += 1
            if self.valid_streak >= cfg.TRACK_START_CONFIRM_FRAMES:
                self.path_locked = True
            if single_side is not None:
                self.single_lane_side = single_side
                self.single_lane_frames += 1
                status = f"SINGLE_{single_side}"
            elif cfg.CENTER_LINE_MODE:
                self.single_lane_side = None
                self.single_lane_frames = 0
                status = (
                    "CENTER_TRACKING"
                    if self.path_locked
                    else "CENTER_CONFIRMING"
                )
            else:
                self.single_lane_side = None
                self.single_lane_frames = 0
                status = (
                    "TRACKING" if self.path_locked else "CONFIRMING"
                )
        else:
            self.lost_count += 1
            self.valid_streak = 0
            if self.lost_count > cfg.MEMORY_FRAMES:
                self.path_locked = False
                self.path_coefficients = None
                self.single_lane_side = None
                self.single_lane_frames = 0
            status = (
                "CENTER_MEMORY"
                if cfg.CENTER_LINE_MODE
                and self.path_locked
                and self.path_coefficients is not None
                and self.lost_count <= cfg.MEMORY_FRAMES
                else "MEMORY"
                if self.path_locked
                and self.path_coefficients is not None
                and self.lost_count <= cfg.MEMORY_FRAMES
                else "CENTER_LOST"
                if cfg.CENTER_LINE_MODE
                else "LOST"
            )

        usable_path = (
            self.path_locked
            and
            self.path_coefficients is not None
            and self.lost_count <= cfg.MEMORY_FRAMES
        )
        if usable_path:
            (
                raw_error,
                heading_error,
                curvature,
                control_x,
                control_errors,
                guard,
                left_guard,
                curve_margin,
                near_right,
                lateral_safety,
                preview_error,
                preview_x,
                preview_weight,
                corridor_steer,
                hard_boundary,
                left_anchor_error,
                left_boundary_side,
                offset_lane_width_px,
            ) = self._calculate_control(
                height, width, measurements, status
            )
            self.filtered_error = (
                cfg.ERROR_FILTER * self.filtered_error
                + (1.0 - cfg.ERROR_FILTER) * raw_error
            )
            if abs(self.filtered_error) < cfg.STEER_DEADBAND:
                self.filtered_error = 0.0
            derivative_rate = (
                self.filtered_error - self.previous_error
            ) / max(delta_time, 0.01)
            self.previous_error = self.filtered_error
            self.filtered_derivative = (
                cfg.DERIVATIVE_FILTER * self.filtered_derivative
                + (1.0 - cfg.DERIVATIVE_FILTER) * derivative_rate
            )

            target_steer = (
                cfg.STEER_CENTER
                - self.filtered_error * cfg.STEER_KP
                - self.filtered_derivative * cfg.STEER_KD
                - corridor_steer
            )
            if status == "MEMORY":
                neutral_target = (
                    cfg.STEER_CENTER
                    + (self.previous_steer - cfg.STEER_CENTER)
                    * cfg.MEMORY_STEER_RETAIN_RATIO
                )
                if self.previous_steer < cfg.STEER_CENTER:
                    target_steer = max(target_steer, neutral_target)
                elif self.previous_steer > cfg.STEER_CENTER:
                    target_steer = min(target_steer, neutral_target)
            if hard_boundary == 1:
                target_steer = max(
                    target_steer,
                    cfg.STEER_CENTER + cfg.BOUNDARY_HARD_STEER_OFFSET,
                )
            elif hard_boundary == -1:
                target_steer = min(
                    target_steer,
                    cfg.STEER_CENTER - cfg.BOUNDARY_HARD_STEER_OFFSET,
                )
            target_steer = clamp(
                target_steer, cfg.STEER_RIGHT, cfg.STEER_LEFT
            )
            # FPS가 달라져도 같은 물리 속도로 조향하도록 시간 기준으로 제한합니다.
            if hard_boundary != 0:
                steer_rate = cfg.BOUNDARY_HARD_STEER_RATE
            else:
                speed_span = max(1.0, cfg.BASE_SPEED - cfg.CURVE_SPEED)
                speed_ratio = clamp(
                    (self.previous_speed - cfg.CURVE_SPEED) / speed_span,
                    0.0,
                    1.0,
                )
                steer_rate = cfg.STEER_RATE_PER_SECOND * (
                    1.0
                    - speed_ratio
                    * cfg.HIGH_SPEED_STEER_RATE_REDUCTION
                    * (
                        1.0 - clamp(
                            curvature / cfg.CURVE_SPEED_TRIGGER,
                            0.0,
                            1.0,
                        )
                    )
                )
            maximum_step = steer_rate * delta_time
            self.steer_float = clamp(
                target_steer,
                self.previous_steer - maximum_step,
                self.previous_steer + maximum_step,
            )
            steering = int(round(clamp(
                self.steer_float,
                cfg.STEER_RIGHT,
                cfg.STEER_LEFT,
            )))

            severity = (
                abs(self.filtered_error) * cfg.ERROR_SPEED_GAIN
                + abs(heading_error) * cfg.HEADING_SPEED_GAIN
                + curvature * cfg.CURVATURE_SPEED_GAIN
            )
            target_speed = int(round(clamp(
                cfg.BASE_SPEED - severity,
                cfg.CURVE_SPEED,
                cfg.BASE_SPEED,
            )))
            if curvature >= cfg.CURVE_SPEED_TRIGGER:
                target_speed = min(target_speed, cfg.CURVE_MAX_SPEED)
            if status == "MEMORY":
                target_speed = min(target_speed, cfg.RETURN_SPEED)
            elif status in ("SINGLE_LEFT", "SINGLE_RIGHT", "CENTER_TRACKING", "CENTER_CONFIRMING"):
                if status in ("SINGLE_LEFT", "SINGLE_RIGHT"):
                    target_speed = min(target_speed, cfg.SINGLE_LANE_SPEED)
            self.speed_float = (
                0.78 * self.speed_float + 0.22 * target_speed
            )
            speed = int(round(self.speed_float))
            if 0 < speed < cfg.MIN_EFFECTIVE_DRIVE_SPEED:
                speed = cfg.MIN_EFFECTIVE_DRIVE_SPEED
                self.speed_float = float(speed)
        else:
            raw_error = 0.0
            heading_error = 0.0
            curvature = 0.0
            control_x = np.asarray([], dtype=np.float64)
            control_errors = np.asarray([], dtype=np.float64)
            guard = 0.0
            left_guard = 0.0
            curve_margin = 0.0
            near_right = None
            lateral_safety = 0.0
            preview_error = 0.0
            preview_x = None
            preview_weight = 0.0
            corridor_steer = 0.0
            hard_boundary = 0
            left_anchor_error = 0.0
            left_boundary_side = None
            steering = self.previous_steer
            # 최초 차선 확인 전 또는 메모리 허용시간 이후에는 출발하지 않습니다.
            speed = 0
            self.filtered_error *= 0.90
            self.filtered_derivative *= 0.75
            self.previous_error *= 0.90

        self.previous_steer = steering
        self.previous_speed = speed
        return {
            "speed": speed,
            "steering": steering,
            "status": status,
            "near_source": self._source_near(
                measurements, height, cfg.CONTROL_Y_RATIOS[0]
            ),
            "far_source": self._source_near(
                measurements, height, cfg.CONTROL_Y_RATIOS[2]
            ),
            "center_error": raw_error,
            "heading_error": heading_error,
            "curvature": curvature,
            "guard": guard,
            "left_guard": left_guard,
            "left_anchor_error": left_anchor_error,
            "curve_margin": curve_margin,
            "near_right": near_right,
            "lateral_safety": lateral_safety,
            "preview_error": preview_error,
            "preview_x": preview_x,
            "preview_weight": preview_weight,
            "corridor_steer": corridor_steer,
            "hard_boundary": hard_boundary,
            "single_lane_side": self.single_lane_side,
            "single_lane_frames": self.single_lane_frames,
            "width_model_age": self.width_model_age,
            "lane_offset_lanes": self._lane_offset_lanes,
            "offset_lane_width_px": offset_lane_width_px,
            "left_boundary_side": left_boundary_side,
            "confidence": confidence,
            "control_x": control_x,
            "control_errors": control_errors,
            "roi": roi,
            "mask": mask,
            "threshold": threshold,
            "measurements": measurements,
            "inliers": inliers,
            "path_coefficients": (
                None
                if self.path_coefficients is None
                else self.path_coefficients.copy()
            ),
        }

    @staticmethod
    def _source_near(measurements, height, y_ratio):
        if not measurements:
            return "NONE"
        target_y = height * y_ratio
        item = min(
            measurements, key=lambda value: abs(value["y"] - target_y)
        )
        return item["source"]

    @staticmethod
    def draw_debug(frame, result):
        """검출 중앙점, 곡선 경로와 제어 목표점을 영상에 표시합니다."""
        roi = result["roi"]
        height, width = result["mask"].shape

        for item in result["measurements"]:
            y = int(item["y"])
            if item["left"] is not None:
                cv2.circle(
                    roi, (int(item["left"]), y), 3, (255, 255, 0), -1
                )
            if item["right"] is not None:
                cv2.circle(
                    roi, (int(item["right"]), y), 3, (0, 0, 255), -1
                )
            color = (
                (0, 255, 0)
                if item["source"] == "CENTER"
                else (0, 255, 255)
                if item["source"] == "BOTH"
                else (0, 165, 255)
            )
            cv2.circle(
                roi, (int(item["center"]), y), 4, color, -1
            )

        coefficients = result["path_coefficients"]
        if coefficients is not None:
            path_points = []
            for y in np.linspace(
                height * cfg.TRACK_TOP_RATIO,
                height * cfg.TRACK_BOTTOM_RATIO,
                80,
            ):
                forward = 1.0 - y / max(1.0, height - 1)
                x = int(round(np.polyval(coefficients, forward)))
                if 0 <= x < width:
                    path_points.append((x, int(round(y))))
            if len(path_points) >= 2:
                cv2.polylines(
                    roi,
                    [np.asarray(path_points, dtype=np.int32)],
                    False,
                    (255, 0, 255),
                    3,
                )

            for index, y_ratio in enumerate(cfg.CONTROL_Y_RATIOS):
                y = int(height * y_ratio)
                # 파란 점은 가까운 위치의 차량 중심축 기준만 표시합니다.
                # 중간·먼 지점은 고정 X가 아니라 경로 방향 계산에 사용합니다.
                if index == 0:
                    target_x = int(scale_x(
                        cfg.CENTER_LINE_TARGET_X
                        if cfg.CENTER_LINE_MODE
                        else cfg.CONTROL_TARGET_X[index],
                        width,
                    ))
                    cv2.circle(
                        roi, (target_x, y), 5, (255, 0, 0), -1
                    )
                if index < len(result["control_x"]):
                    cv2.circle(
                        roi,
                        (int(result["control_x"][index]), y),
                        6,
                        (0, 255, 0),
                        2,
                    )
            preview_x = result.get("preview_x")
            if preview_x is not None:
                cv2.circle(
                    roi,
                    (
                        int(round(preview_x)),
                        int(round(height * cfg.PREVIEW_Y_RATIO)),
                    ),
                    7,
                    (255, 255, 0),
                    2,
                )
        return frame
