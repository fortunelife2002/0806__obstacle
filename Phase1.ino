#include <Car_Library.h>

// =============================================================================
// 하드웨어 핀
// =============================================================================
const int drive1_IN1 = 8;
const int drive1_IN2 = 9;
const int drive2_IN1 = 6;
const int drive2_IN2 = 7;
const int steer_IN1 = 11;
const int steer_IN2 = 3;
const int potPin = A5;

// =============================================================================
// 수신 명령과 조향 범위
// =============================================================================
int drive_dir = 0;
int drive_speed = 0;
int target_angle = 157;

const int STEER_CENTER = 157;
const int STEER_MIN = 100;
const int STEER_MAX = 210;

// =============================================================================
// 조향 안정화 설정
// =============================================================================
// HOLD 진입 범위와 재출발 범위를 다르게 해 목표 근처 좌우 반전을 방지한다.
const int STEER_STOP_TOLERANCE = 3;
const int STEER_RESTART_TOLERANCE = 6;

// 목표 근처 출력이 너무 크면 관성으로 목표를 지나치므로 단계별로 사용한다.
const int STEER_NEAR_POWER = 75;
const int STEER_MIDDLE_POWER = 105;
const int STEER_MAX_POWER = 150;

const int STEER_NEAR_ERROR = 8;
const int STEER_MIDDLE_ERROR = 18;

// 가변저항 저역통과 필터: 새 값 25%, 기존 값 75%
const int POT_FILTER_OLD_WEIGHT = 3;
const int POT_FILTER_DIVISOR = 4;

int filtered_angle = STEER_CENTER;
bool steering_active = false;

// -1: backward, 0: hold, 1: forward
int previous_steer_direction = 0;

// =============================================================================
// 후륜 좌우 편차 보정
// 직진 시험 전에는 0으로 유지하고 실제 편차가 확인된 경우에만 조정한다.
// =============================================================================
const int LEFT_MOTOR_TRIM = 0;
const int RIGHT_MOTOR_TRIM = 0;

// =============================================================================
// 통신 및 안전
// =============================================================================
char receiveBuffer[32];
byte receiveIndex = 0;
unsigned long lastCommandTime = 0;

// 신호등/장애물 처리가 추가돼도 순간적인 영상 지연에 정지하지 않도록 여유 확보
const unsigned long COMMAND_TIMEOUT_MS = 1200;


void receiveCommand()
{
  while (Serial.available() > 0)
  {
    char receivedChar = Serial.read();

    if (receivedChar == '\n')
    {
      receiveBuffer[receiveIndex] = '\0';

      int new_dir;
      int new_speed;
      int new_angle;

      if (
        sscanf(
          receiveBuffer,
          "%d,%d,%d",
          &new_dir,
          &new_speed,
          &new_angle
        ) == 3
      )
      {
        drive_dir = constrain(new_dir, 0, 2);
        drive_speed = constrain(new_speed, 0, 255);
        target_angle = constrain(
          new_angle,
          STEER_MIN,
          STEER_MAX
        );
        lastCommandTime = millis();
      }

      receiveIndex = 0;
    }
    else if (receivedChar != '\r')
    {
      if (receiveIndex < sizeof(receiveBuffer) - 1)
      {
        receiveBuffer[receiveIndex++] = receivedChar;
      }
      else
      {
        // 손상된 긴 패킷은 폐기한다.
        receiveIndex = 0;
      }
    }
  }
}


int readFilteredSteeringAngle()
{
  int raw_angle = potentiometer_Read(potPin);
  filtered_angle = (
    filtered_angle * POT_FILTER_OLD_WEIGHT
    + raw_angle
  ) / POT_FILTER_DIVISOR;
  return filtered_angle;
}


int calculateSteeringPower(int absolute_error)
{
  if (absolute_error <= STEER_NEAR_ERROR)
  {
    return STEER_NEAR_POWER;
  }

  if (absolute_error <= STEER_MIDDLE_ERROR)
  {
    return STEER_MIDDLE_POWER;
  }

  // 큰 오차에서는 105~150 범위에서 비례적으로 증가한다.
  return constrain(
    STEER_MIDDLE_POWER
    + (absolute_error - STEER_MIDDLE_ERROR) * 3,
    STEER_MIDDLE_POWER,
    STEER_MAX_POWER
  );
}


void holdSteering()
{
  motor_hold(steer_IN1, steer_IN2);
  previous_steer_direction = 0;
}


void controlSteering()
{
  int current_angle = readFilteredSteeringAngle();
  int steering_error = target_angle - current_angle;
  int absolute_error = abs(steering_error);

  // 움직이는 중에는 작은 범위에 도착하면 정지한다.
  if (steering_active && absolute_error <= STEER_STOP_TOLERANCE)
  {
    steering_active = false;
    holdSteering();
    return;
  }

  // 정지 중에는 오차가 충분히 커질 때만 다시 움직인다.
  if (!steering_active)
  {
    if (absolute_error < STEER_RESTART_TOLERANCE)
    {
      holdSteering();
      return;
    }
    steering_active = true;
  }

  int requested_direction = steering_error > 0 ? 1 : -1;

  // 즉시 정방향/역방향을 바꾸지 않고 한 제어 주기 브레이크한다.
  if (
    previous_steer_direction != 0
    && requested_direction != previous_steer_direction
  )
  {
    holdSteering();
    return;
  }

  int steer_power = calculateSteeringPower(absolute_error);

  if (requested_direction > 0)
  {
    motor_forward(steer_IN1, steer_IN2, steer_power);
  }
  else
  {
    motor_backward(steer_IN1, steer_IN2, steer_power);
  }

  previous_steer_direction = requested_direction;
}


void holdDriveMotors()
{
  motor_hold(drive1_IN1, drive1_IN2);
  motor_hold(drive2_IN1, drive2_IN2);
}


void controlDriveMotors()
{
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)
  {
    drive_dir = 0;
    drive_speed = 0;
  }

  if (drive_dir == 0 || drive_speed <= 0)
  {
    holdDriveMotors();
    return;
  }

  int left_speed = constrain(
    drive_speed + LEFT_MOTOR_TRIM,
    0,
    255
  );
  int right_speed = constrain(
    drive_speed + RIGHT_MOTOR_TRIM,
    0,
    255
  );

  if (drive_dir == 1)
  {
    motor_forward(drive1_IN1, drive1_IN2, left_speed);
    motor_forward(drive2_IN1, drive2_IN2, right_speed);
  }
  else if (drive_dir == 2)
  {
    motor_backward(drive1_IN1, drive1_IN2, left_speed);
    motor_backward(drive2_IN1, drive2_IN2, right_speed);
  }
  else
  {
    holdDriveMotors();
  }
}


void setup()
{
  Serial.begin(9600);

  pinMode(drive1_IN1, OUTPUT);
  pinMode(drive1_IN2, OUTPUT);
  pinMode(drive2_IN1, OUTPUT);
  pinMode(drive2_IN2, OUTPUT);
  pinMode(steer_IN1, OUTPUT);
  pinMode(steer_IN2, OUTPUT);

  // 시작 시 실제 센서값으로 필터를 초기화해 중앙값으로 서서히 끌려가는 현상 방지
  filtered_angle = potentiometer_Read(potPin);
  target_angle = STEER_CENTER;
  lastCommandTime = millis();

  holdSteering();
  holdDriveMotors();
}


void loop()
{
  receiveCommand();
  controlSteering();
  controlDriveMotors();

  delay(10);
}
