"""
-------------------------------------------------------------------
  FILE NAME: Function_Library.py
  Copyright: Sungkyunkwan University, Automation Lab.
-------------------------------------------------------------------
  This file is included library class for below subject.
  1) Arduino
  2) LiDAR
  3) Camera
-------------------------------------------------------------------
  Authors: Jonghun Kim, YoungSoo Do, SungBhin Oh, HyeongKeun Hong

  Generated: 2022-11-10
  Revised: 2022-11-18
-------------------------------------------------------------------
  If you find some wrong code, plz contact me(Main Author: Jonghun Kim).
-------------------------------------------------------------------
  You should never modify this file during workshop exercise.
-------------------------------------------------------------------
"""

import sys
import cv2  # pip install opencv : 컴퓨터 비전(카메라 영상 처리)을 위한 핵심 라이브러리
import time  # 시간 지연(delay) 및 타이머 기능을 위한 라이브러리
import serial  # pip install serial : 아두이노와의 USB 시리얼 통신을 위한 라이브러리
import numpy as np  # pip install numpy : 영상 픽셀 데이터나 라이다 데이터를 고속으로 계산하기 위한 수학 연산 라이브러리
from rplidar import RPLidar  # pip install rplidar-roboticia : Slamtec 2D 라이다(RPLidar) 제어 전용 라이브러리

# 터미널 창에 긴 배열 데이터를 출력할 때 중간에 잘리지 않고 끝까지 나오도록 설정
np.set_printoptions(threshold=sys.maxsize, linewidth=150)

"""------------------Arduino Variable------------------"""
WAIT_TIME = 2  # 아두이노와 시리얼 포트 연결 후 통신이 안정화될 때까지 기다리는 물리적 대기 시간(초)
"""----------------------------------------------------"""

"""-------------------LIDAR Variable-------------------"""
SCAN_TYPE = "normal"  # 라이다 스캔 모드 설정 (일반 스캔)
SAMPLE_RATE = 10  # 노이즈 방지를 위해 한 번에 읽어 들일 최소 데이터 개수 (이것보다 적게 읽히면 무시)
MAX_BUFFER_SIZE = 3000  # 시리얼 통신 과정에서 스캔 데이터를 임시로 담아둘 버퍼의 최대 크기
MIN_DISTANCE = 0  # 라이다 센서가 취급할 최소 거리 (0보다 가까운 비정상 데이터는 걸러냄)
"""----------------------------------------------------"""

"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30  # 차선 인식 시, 두 선분이 같은 선상에 있는지 오차를 허용해 주는 픽셀 범위
SATURATION = 150  # HSV 색상 공간에서 채도(색의 진함) 기준값. 이 값 이상이어야 유효한 색상으로 판단
FORWARD_THRESHOLD = 0.3  # 차선 기울기 기준값. 기울기의 절댓값이 이 값보다 작으면 직진(FORWARD)으로 판정
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)  # 색상 상태를 나타내는 내부 상수 (인덱스 번호 역할)
FORWARD, LEFT, RIGHT = (0, 1, 2)  # 주행 방향 상태를 나타내는 내부 상수
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")  # 색상 인덱스를 문자열로 변환할 때 사용할 튜플
DIRECTION = ("FORWARD", "LEFT", "RIGHT")  # 방향 인덱스를 문자열로 변환할 때 사용할 튜플

# 신호등 인식을 위한 HSV 색상 임계값 (Hue, 색상 값) 설정
# [최소치, 최대치] 형태. 예: RED는 4미만 또는 176초과의 H값을 가짐 (빨간색은 HSV 띠의 양 끝에 걸쳐 있음)
HUE_THRESHOLD = ([4, 176], [40, 80], [110, 130], [20, 40])
"""-----------------------------------------------------"""

"""
-------------------------------------------------------------------
  CLASS PURPOSE: Arduino Exercise Library
  Author: SungBhin Oh
  Revised: 2022-11-14
-------------------------------------------------------------------
"""


# noinspection PyMethodMayBeStatic
class libARDUINO(object):
    def __init__(self):
        self.port = None  # 아두이노가 연결된 포트 번호 (예: 'COM5')
        self.baudrate = None  # 통신 속도 (보통 9600 사용)
        self.wait_time = WAIT_TIME  # 통신 안정화 대기 시간 (2초)

    # 파이썬과 아두이노의 물리적 USB 연결을 시작하는 함수
    def init(self, port, baudrate):
        ser = serial.Serial()  # 시리얼 통신 객체 생성
        ser.port, self.port = port, port
        ser.baudrate, self.baudrate = baudrate, baudrate
        ser.open()  # 포트 개방 (통화 연결)
        time.sleep(self.wait_time)  # 아두이노가 리셋되고 준비될 때까지 2초 대기
        return ser  # 연결된 통신 객체(전화기)를 반환하여 메인 코드에서 쓸 수 있게 함


"""
-------------------------------------------------------------------
  CLASS PURPOSE: LiDAR Sensor Exercise Library
  Author: YoungSoo Do
  Revised: 2022-11-18
-------------------------------------------------------------------
"""


class libLIDAR(object):
    def __init__(self, port):
        self.rpm = 0  # 라이다 모터 회전 속도 저장 변수
        self.lidar = RPLidar(port)  # 입력받은 포트(COM3 등)로 라이다 객체 생성
        self.scan = []  # 스캔된 데이터를 담을 리스트

    def init(self):  # 라이다의 시리얼 넘버, 펌웨어 버전 등 기기 정보 출력 (에러 발생 시 이 부분을 패스해야 함)
        info = self.lidar.get_info()
        print(info)

    def getState(self):  # 라이다의 현재 센서 상태(정상/에러 등) 체크 및 출력
        health = self.lidar.get_health()
        print(health)

    def scanning(self):  # 라이다 모터를 가동하고 실시간 데이터를 뿜어내는 핵심 제너레이터(yield) 함수
        scan_list = []
        iterator = self.lidar.iter_measures(SCAN_TYPE, MAX_BUFFER_SIZE)  # 실시간 데이터 측정 루프 시작

        for new_scan, quality, angle, distance in iterator:
            if new_scan:  # 360도 한 바퀴 스캔이 끝났고 새로운 바퀴가 시작되었다면
                if len(scan_list) > SAMPLE_RATE:  # 모인 데이터가 10개(SAMPLE_RATE) 이상일 때만 (노이즈 방지)
                    np_data = np.array(list(scan_list))
                    yield np_data[:, 1:]  # 퀄리티 값은 버리고 [각도, 거리] 배열만 반환
                scan_list = []  # 반환 후 다음 바퀴를 위해 리스트 초기화

            if distance > MIN_DISTANCE:  # 0mm보다 큰 (정상적인) 거리 데이터만
                scan_list.append((quality, angle, distance))  # 리스트에 차곡차곡 모음

    def stop(self):  # 코드 종료 시 라이다 모터가 계속 돌지 않도록 완전히 정지시키고 포트 닫음
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def setRPM(self, rpm):  # 라이다 모터 회전 속도 조절
        self.lidar.motor_speed = rpm

    def getRPM(self):  # 현재 라이다 모터 회전 속도 확인
        return self.lidar.motor_speed

    def getAngleRange(self, scan, minAngle, maxAngle):  # 전체 스캔 데이터 중 특정 각도(FOV) 내부의 점들만 추출
        data = np.array(scan)
        condition = np.where((data[:, 0] < maxAngle) & (data[:, 0] > minAngle))
        return data[condition]

    def getDistanceRange(self, scan, minDist, maxDist):  # 전체 스캔 데이터 중 특정 거리 내부의 점들만 추출
        data = np.array(scan)
        condition = np.where((data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]

    def getAngleDistanceRange(self, scan, minAngle, maxAngle, minDist, maxDist):  # 특정 각도이면서 특정 거리 내에 있는 점만 추출
        data = np.array(scan)
        condition = np.where(
            (data[:, 0] < maxAngle) & (data[:, 0] > minAngle) & (data[:, 1] < maxDist) & (data[:, 1] > minDist))
        return data[condition]
    
    def get_far_distance(self, scan, minAngle, maxAngle): #장애물을 피해 멀리(뚫린) 곳으로 이동 시 유용
        datas = self.getAngleRange(scan, minAngle, maxAngle)
        max_idx = datas[:, 1].argmax()
        return datas[max_idx]

    def get_near_distance(self, scan, minAngle, maxAngle): #가까이 있는 장애물을 인식해 즉시 정지하는데 유용    datas = self.getAngleRange(scan, minAngle, maxAngle)
        min_idx = datas[:, 1].argmin()
        return datas[min_idx]


"""
-------------------------------------------------------------------
  CLASS PURPOSE: Camera Sensor Exercise Library
  Author: Jonghun Kim
  Revised: 2022-11-12
-------------------------------------------------------------------
"""


# noinspection PyMethodMayBeStatic
class libCAMERA(object):
    def __init__(self):
        self.capnum = 0  # 연결된 카메라 대수
        self.row, self.col, self.dim = (0, 0, 0)  # 이미지의 세로, 가로, 색상 채널 크기 변수 초기화

    def loop_break(self):  # 키보드 'q'를 누르면 프로그램(카메라 영상)을 강제로 종료하는 안전장치
        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Camera Reading is ended.")
            return True
        else:
            return False

    def file_read(self, img_path):  # 파일 경로에 있는 이미지를 불러와서 NumPy 배열로 변환
        return np.array(cv2.imread(img_path))

    def rgb_conversion(self, img):  # OpenCV 기본 색상인 BGR을 사람 눈에 익숙한 RGB로 변환
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)

    def hsv_conversion(self, img):  # 색상(H), 채도(S), 명도(V) 형태의 데이터로 변환 (조명 변화에 강해서 객체 인식에 필수적)
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)

    def gray_conversion(self, img):  # 컬러 영상을 흑백(Gray)으로 변환 (연산량 감소 및 형태/외곽선 파악에 유리)
        return cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)

    def color_extract(self, img, idx):  # 지정된 색상 채널(R, G, B 중 하나)만 남기고 나머지는 까맣게 지워버림
        result = img.copy()

        for i in range(RED + GREEN + BLUE):  # 0, 1, 2 루프
            if i != idx:  # 선택한 색상이 아니면
                result[:, :, i] = np.zeros([self.row, self.col])  # 0(검은색)으로 덮어씀

        return result

    def extract_rgb(self, img, print_enable=False):  # 이미지를 R, G, B 3개의 독립된 채널로 쪼개는 함수
        self.row, self.col, self.dim = img.shape
        img = self.rgb_conversion(img)

        # 각 색상 채널 분리
        img_red = self.color_extract(img, RED)
        img_green = self.color_extract(img, GREEN)
        img_blue = self.color_extract(img, BLUE)

        if print_enable:  # True로 설정하면 화면에 R, G, B 채널을 각각 그래프처럼 띄워줌
            plt.figure(figsize=(12, 4))
            imgset = [img_red, img_green, img_blue]
            imglabel = ["RED", "GREEN", "BLUE"]

            for idx in range(RED + GREEN + BLUE):
                plt.subplot(1, 3, idx + 1)
                plt.xlabel(imglabel[idx])
                plt.imshow(imgset[idx])
            plt.show()

        return img_red[:, :, RED], img_green[:, :, GREEN], img_blue[:, :, BLUE]

    def initial_setting(self, cam0port=0, cam1port=1, capnum=1):  # 사용할 웹캠의 하드웨어 포트를 열고 초기화
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        self.capnum = capnum  # 사용할 카메라 개수 셋팅 (1대 또는 2대)

        # 카메라 대수에 맞춰 포트를 열고 연결 확인 메시지 출력
        if capnum == 1:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)  # 윈도우 환경(DSHOW)에서 첫 번째 카메라 오픈
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")

            channel1 = cv2.VideoCapture(cv2.CAP_DSHOW + cam1port)  # 두 번째 카메라 오픈
            if channel1.isOpened():
                print("Camera Channel1 is enabled!")

        return channel0, channel1  # 열린 카메라 객체 반환

    def camera_read(self, cap1, cap2=None):  # 연결된 카메라로부터 현재 찰나의 프레임(사진)을 1장 읽어옴
        result, capset = [], [cap1, cap2]

        for idx in range(0, self.capnum):
            ret, frame = capset[idx].read()  # ret은 성공여부(True/False), frame은 이미지 데이터
            result.extend([ret, frame])

        return result

    def image_show(self, frame0, frame1=None):  # 읽어온 이미지를 화면에 팝업 창으로 띄워줌
        if frame1 is None:
            cv2.imshow('frame0', frame0)
        else:
            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)

    def color_filtering(self, img, roi=None, print_enable=False):  # 특정 색상(빨강,초록,노랑)만 남기고 화면 전체를 검게 마스킹하는 함수
        self.row, self.col, self.dim = img.shape

        hsv_img = self.hsv_conversion(img)  # 조명 간섭을 피하기 위해 HSV로 변환
        h, s, v = cv2.split(hsv_img)  # H(색상), S(채도), V(명도) 분리

        s_cond = s > SATURATION  # 채도가 설정값(150)보다 높은, 즉 색이 뚜렷한 곳만 True

        # 빨간색은 스펙트럼 양 끝(0쪽, 180쪽)에 나뉘어 있어서 '또는(|)' 조건 사용
        if roi is RED:
            h_cond = (h < HUE_THRESHOLD[roi][0]) | (h > HUE_THRESHOLD[roi][1])
        # 나머지 색상은 지정된 H값 범위 안에 있는지 '그리고(&)' 조건 사용
        else:
            h_cond = (h > HUE_THRESHOLD[roi][0]) & (h < HUE_THRESHOLD[roi][1])

        # 색상이 맞지 않거나 채도가 낮은 픽셀의 명도(V)를 0으로 만들어 까맣게 지워버림
        v[~h_cond], v[~s_cond] = 0, 0
        hsv_image = cv2.merge([h, s, v])  # 수정된 H, S, V를 다시 합침
        result = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)  # OpenCV 출력을 위해 BGR로 원상복구

        if print_enable:
            self.image_show(result)  # 필터링 결과를 화면에 보여줌

        return result

    def gaussian_blurring(self, img, kernel_size=(None, None)):  # 화면을 뿌옇게 만들어 자잘한 노이즈(먼지 등)를 제거하는 블러 처리
        return cv2.GaussianBlur(img.copy(), kernel_size, 0)

    def canny_edge(self, img, lth, hth):  # 이미지에서 픽셀 밝기가 급격하게 변하는 부분(외곽선/차선)만 하얀 실선으로 따냄
        return cv2.Canny(img.copy(), lth, hth)

    def histogram_equalization(self, gray):  # 영상의 명암 대비를 극대화시켜 선명하게 만듦 (너무 밝거나 어두운 곳 보정)
        return cv2.equalizeHist(gray)

    def hough_transform(self, img, rho=None, theta=None, threshold=None, mll=None, mlg=None, mode="lineP"):
        # 점들을 모아서 의미 있는 선(차선)이나 원(신호등)을 수학적으로 찾아내는 허프(Hough) 변환 함수
        if mode == "line":
            return cv2.HoughLines(img.copy(), rho, theta, threshold)
        elif mode == "lineP":  # 뚝뚝 끊긴 선분을 하나로 이어주는 확률적 허프 선 변환 (주로 차선 인식에 사용)
            return cv2.HoughLinesP(img.copy(), rho, theta, threshold, lines=np.array([]),
                                   minLineLength=mll, maxLineGap=mlg)
        elif mode == "circle":  # 허프 원 변환 (주로 동그란 신호등 렌즈를 찾을 때 사용)
            return cv2.HoughCircles(img.copy(), cv2.HOUGH_GRADIENT, dp=1, minDist=80,
                                    param1=200, param2=10, minRadius=40, maxRadius=100)

    def morphology(self, img, kernel_size=(None, None), mode="opening"):
        # 영상의 미세한 구멍을 메우거나, 자잘한 찌꺼기를 없애는 형태학적 연산
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)

        if mode == "opening":  # 깎았다가 부풀리기 (자잘한 노이즈 제거)
            dst = cv2.erode(img.copy(), kernel)
            return cv2.dilate(dst, kernel)
        elif mode == "closing":  # 부풀렸다가 깎기 (선이나 면 안쪽의 구멍 메우기)
            dst = cv2.dilate(img.copy(), kernel)
            return cv2.erode(dst, kernel)
        elif mode == "gradient":  # 외곽선 추출
            return cv2.morphologyEx(img.copy(), cv2.MORPH_GRADIENT, kernel)

    def point_analyze(self, gray, line, point_gap, len_threshold):
        # 찾은 선이 진짜 흰색/노란색 차선인지, 아니면 그냥 바닥의 그림자인지 픽셀 밝기 차이를 비교하여 판독하는 함수
        disparity = [0, 0]

        for idx in range(2):
            # 선을 기준으로 양옆(위아래) 픽셀의 좌표를 구함
            yplus = line[idx + 1] + point_gap if line[idx + 1] + point_gap < self.row else self.row - 1
            yminus = line[idx + 1] - point_gap if line[idx + 1] - point_gap >= 0 else 0

            # 이미지를 벗어나면 중지
            if yplus < 0 or yminus >= self.row:
                break
            elif yplus >= self.row or yminus < 0:
                break

            # 양쪽 픽셀 밝기의 차이(절댓값)를 계산
            disparity[idx] = np.abs(gray[yplus][line[idx]] - gray[yminus][line[idx]])

        # 양옆의 밝기 차이가 기준값(len_threshold)보다 크면 확연한 경계가 있는 진짜 차선으로 인정 (True)
        if np.average(disparity) > len_threshold:
            return True
        else:
            return False

    def object_detection(self, img, sample=0, mode="circle", print_enable=False):
        # [신호등 인식 메인 로직] 특정 색상 필터링 후, 동그란 원을 찾아서 그것이 무슨 색의 신호등인지 판단
        result = None
        replica = img.copy()  # 원본 보존을 위한 복사본

        for color in (RED, YELLOW, GREEN):  # 3가지 색상에 대해 반복 검사
            extract = self.color_filtering(img, roi=color, print_enable=True)  # 해당 색상만 남김
            gray = self.gray_conversion(extract)  # 원 검출을 위해 흑백 변환
            circles = self.hough_transform(gray, mode=mode)  # 허프 원 검출 알고리즘 적용

            if circles is not None:  # 화면에 동그란 원이 발견되었다면
                for circle in circles[0]:
                    center, count = (int(circle[0]), int(circle[1])), 0  # 원의 중심점 좌표(X, Y) 획득

                    hsv_img = self.hsv_conversion(img)
                    h, s, v = cv2.split(hsv_img)

                    # 찾은 원의 중심점 주변(십자가 형태 등)의 픽셀들을 검사해서 실제로 지정한 색상이 맞는지 한 번 더 꼼꼼히 확인
                    for res in range(sample):
                        x, y = int(center[1] - sample / 2), int(center[0] - sample / 2)
                        s_cond = s[x][y] > SATURATION
                        if color is RED:
                            h_cond = (h[x][y] < HUE_THRESHOLD[color][0]) | (h[x][y] > HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count
                        else:
                            h_cond = (h[x][y] > HUE_THRESHOLD[color][0]) & (h[x][y] < HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count

                    # 검사한 주변 픽셀들 중 절반 이상이 타겟 색상이라면 진짜 신호등으로 확정
                    if count > sample / 2:
                        result = COLOR[color]  # "RED", "GREEN" 등의 문자열 결과 저장
                        cv2.circle(replica, center, int(circle[2]), (0, 0, 255), 2)  # 화면에 빨간색 동그라미 그려주기

        if print_enable:  # 화면에 찾은 신호등 결과 표시
            if result is not None:
                print("Traffic Light: ", result)
            self.image_show(replica)

        return result

    def edge_detection(self, img, width=0, height=0, gap=0, threshold=0, print_enable=False):
        # [차선 유지 메인 로직] 화면에서 선을 찾은 뒤, 선의 각도(기울기)를 계산해 핸들을 왼쪽/오른쪽으로 꺾으라는 판정을 내림
        prediction = None
        replica = img.copy()
        self.row, self.col, self.dim = img.shape

        # 1. 영상 최적화: 흑백화 -> 평활화 -> 찌꺼기 제거 -> 블러 처리
        gray_scale = self.gray_conversion(img)
        hist = self.histogram_equalization(gray_scale)
        dst = self.morphology(hist, (2, 2), mode="opening")
        blurring = self.gaussian_blurring(dst, (5, 5))

        # 2. 캐니 에지로 픽셀 경계선 추출
        canny = self.canny_edge(blurring, 100, 200)

        # 3. 허프 변환으로 경계선을 잇는 진짜 직진 '선분' 찾기
        lines = self.hough_transform(canny, 1, np.pi / 180, 50, 10, 20, mode="lineP")

        if lines is not None:  # 선이 하나라도 발견되었다면
            new_lines, real_lines = [], []
            for line in lines:
                xa, ya, xb, yb = line[0]  # 찾은 선의 시작점(xa,ya)과 끝점(xb,yb) 좌표

                # 선의 가로/세로 길이를 설정값과 비교해 유효한 길이의 차선인지 검사
                if np.abs(yb - ya) > height and np.abs(xb - xa) < width:
                    if self.point_analyze(blurring, line[0], gap, threshold):  # 그림자가 아닌 진짜 차선인지 검사

                        for idx in range(len(new_lines)):
                            # 유사한 각도를 가진 중복된 선분들을 하나로 묶기 위한 오차 검사
                            if np.abs(new_lines[:][idx][1] - ya) < VARIANCE:
                                if np.abs(new_lines[:][idx][3] - yb) < VARIANCE:

                                    # [핵심] 차선 각도(기울기) 계산 (x 변화량 / y 변화량)
                                    grad = (xb - xa) / -(yb - ya)

                                    # 기울기 값에 따라 차가 어디로 조향해야 할지 판단
                                    if np.abs(grad) < FORWARD_THRESHOLD:  # 기울기가 작으면 수직에 가까운 차선이므로 직진
                                        prediction = FORWARD
                                    elif grad > 0:  # 기울기가 양수면 차선이 오른쪽으로 휘어있으므로 우회전 조향
                                        prediction = RIGHT
                                    elif grad < 0:  # 기울기가 음수면 좌회전 조향
                                        prediction = LEFT

                                    # 찾은 진짜 차선 위에 빨간색(0,0,255) 선 긋기
                                    cv2.line(replica, (xa, ya), (xb, yb), color=[0, 0, 255], thickness=2)
                        new_lines.append([xa, ya, xb, yb])

            if print_enable:
                if prediction is not None:
                    print("Vehicle Direction: ", DIRECTION[prediction])  # 콘솔에 FORWARD, LEFT, RIGHT 출력
                self.image_show(replica)  # 선이 그어진 영상을 화면에 띄움

        return prediction  # 최종 판단된 방향 명령을 반환