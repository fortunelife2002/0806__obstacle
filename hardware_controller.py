"""카메라와 아두이노의 연결 및 통신을 담당합니다."""

import time

import cv2
import serial
from serial.tools import list_ports
import Function_Library as fl

import config as cfg


class HardwareController:
    """카메라 영상을 읽고 차량의 모터 명령을 전송합니다."""

    def __init__(self):
        serial_api = fl.libARDUINO()
        self.camera_api = fl.libCAMERA()
        self.serial = serial_api.init(cfg.ARDUINO_PORT, cfg.BAUD_RATE)
        self.camera, _ = self.camera_api.initial_setting(
            cam0port=cfg.CAMERA_PORT,
            capnum=1,
        )
        self.last_requested_speed = 0
        self.last_sent_speed = 0
        self.boost_until = 0.0
        self._last_reconnect_attempt = 0.0
        self._configure_camera()

    def _write(self, command):
        """시리얼 쓰기 실패(케이블 순간 접촉 불량 등)로 전체 프로그램이
        죽지 않도록 예외를 흡수하고, 너무 잦지 않게(간격 제한) 재연결을
        시도합니다. 실패한 프레임은 그냥 건너뜁니다.
        """
        try:
            self.serial.write(command.encode("utf-8"))
        except serial.SerialException as exc:
            print(f"SERIAL_WRITE_FAILED: {exc}")
            now = time.monotonic()
            if now - self._last_reconnect_attempt >= cfg.SERIAL_RECONNECT_INTERVAL:
                self._last_reconnect_attempt = now
                self._reconnect_serial()

    def _reconnect_serial(self):
        try:
            self.serial.close()
        except Exception:
            pass

        if self._try_open_port(cfg.ARDUINO_PORT):
            print("SERIAL_RECONNECTED")
            return

        # 원래 포트가 그대로 안 돌아오면, USB 재연결로 다른 COM 번호를
        # 받았을 수 있습니다(윈도우가 장치를 새 번호로 재열거하는 경우).
        # 남은 포트를 훑어서 열리는 첫 번째 포트로 넘어갑니다 — 이 차량은
        # 아두이노 하나만 시리얼로 연결하는 구성이라 안전한 가정입니다.
        for port_info in list_ports.comports():
            if port_info.device == cfg.ARDUINO_PORT:
                continue
            if self._try_open_port(port_info.device):
                print(
                    f"SERIAL_RECONNECTED_ON_NEW_PORT: {port_info.device} "
                    f"(config.py의 ARDUINO_PORT={cfg.ARDUINO_PORT!r}를 갱신하세요)"
                )
                return

        print("SERIAL_RECONNECT_FAILED: 열 수 있는 시리얼 포트를 찾지 못했습니다")

    def _try_open_port(self, port):
        try:
            serial_api = fl.libARDUINO()
            self.serial = serial_api.init(port, cfg.BAUD_RATE)
            return True
        except Exception:
            return False

    def _configure_camera(self):
        """신호등 색상이 안정적으로 보이도록 카메라 설정을 고정합니다."""
        if not cfg.LOCK_CAMERA_EXPOSURE:
            return
        if self.camera is None or not hasattr(self.camera, "set"):
            return
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.camera.set(cv2.CAP_PROP_EXPOSURE, cfg.CAMERA_EXPOSURE)
        self.camera.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.camera.set(
            cv2.CAP_PROP_WB_TEMPERATURE,
            cfg.CAMERA_WB_TEMPERATURE,
        )

    def read_frame(self):
        """카메라에서 프레임 한 장을 읽어 반환합니다."""
        _, frame = self.camera_api.camera_read(self.camera)
        return frame

    def drive(self, speed, steering):
        """전진 속도와 조향값을 아두이노로 전송합니다."""
        requested_speed = max(0, int(speed))
        now = time.monotonic()
        if requested_speed <= 0:
            self.boost_until = 0.0
        elif self.last_requested_speed <= 0:
            self.boost_until = now + cfg.DRIVE_START_BOOST_SECONDS

        sent_speed = requested_speed
        if requested_speed > 0 and now < self.boost_until:
            sent_speed = max(sent_speed, cfg.DRIVE_START_BOOST_SPEED)

        command = f"1,{sent_speed},{int(steering)}\n"
        self._write(command)
        self.last_requested_speed = requested_speed
        self.last_sent_speed = sent_speed
        return sent_speed

    def stop(self):
        """모터를 정지하고 조향을 중앙으로 돌립니다."""
        command = f"0,0,{cfg.STEER_CENTER}\n"
        self._write(command)
        self.last_requested_speed = 0
        self.last_sent_speed = 0
        self.boost_until = 0.0

    @staticmethod
    def close():
        """OpenCV가 생성한 모든 화면을 닫습니다."""
        cv2.destroyAllWindows()
