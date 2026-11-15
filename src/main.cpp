#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <HUSKYLENS.h>

// =====================================================
// Arduino Mega 2560 물류 로봇
// HuskyLens 태그 인식 + A4988 스테퍼 모터 제어
//
// 기능
// 1. 태그 ID 1은 무시
// 2. 태그 ID 2~6을 인식하여 해당 방에 적재
// 3. 이미 적재된 방은 재적재 거부
// 4. 1, 3, 5번 방에서는 X축 이동 없음
// 5. 2, 4, 6번 방의 X축 이동량은 상수로 조절
// 6. 상태는 시리얼 모니터로 출력
//
// 주의: 적재 상태는 RAM에 저장되므로 전원을 끄면 초기화됨.
// =====================================================

// ---------------- I2C 설정 ----------------
constexpr uint32_t I2C_CLOCK = 100000UL;
HUSKYLENS huskylens;

// ---------------- 모터 핀 ----------------
const int stepPinX = 2;
const int dirPinX = 3;
const int stepPinZ = 4;
const int dirPinZ = 5;
const int stepPinY = 6;
const int dirPinY = 7;

// ---------------- 모터 설정 ----------------
const long STEPS_PER_REVOLUTION = 3200;
const float MAX_SPEED = 12000.0f;
const float ACCELERATION = 2400.0f;
const float MIN_SPEED = 200.0f;

// Y축 전용 설정
const float Y_MAX_SPEED = 1000.0f;
const float Y_ACCELERATION = 500.0f;
const float Y_MIN_SPEED = 700.0f;

// ---------------- A~H 이동량 ----------------
const float A_Y_FORWARD_REVOLUTIONS = 0.6f;
const float B_Z_FORWARD_REVOLUTIONS = 3.3f;
const float C_Y_REVERSE_REVOLUTIONS = 0.6f;
const float D_Z_REVERSE_REVOLUTIONS = 3.3f;

// 2, 4, 6번 방의 X축 이동량
// 이 값 하나로 이동 거리를 조절
const float EVEN_ROOM_X_REVOLUTIONS = 17.5f;

// 공통 X축 이동은 사용하지 않음
const float E_X_FORWARD_REVOLUTIONS = 0.0f;
const float H_X_REVERSE_REVOLUTIONS = 0.0f;

// 방 1~6의 X축 이동량
// 인덱스 0~5 = 방 번호 1~6
// 1, 3, 5번 방은 X축 이동 없음
const float F_ROOM_X_REVOLUTIONS[6] = {
  0.0f,
  EVEN_ROOM_X_REVOLUTIONS,
  0.0f,
  EVEN_ROOM_X_REVOLUTIONS,
  0.0f,
  EVEN_ROOM_X_REVOLUTIONS
};

const float G_ROOM_X_REVOLUTIONS[6] = {
  0.0f,
  -EVEN_ROOM_X_REVOLUTIONS,
  0.0f,
  -EVEN_ROOM_X_REVOLUTIONS,
  0.0f,
  -EVEN_ROOM_X_REVOLUTIONS
};

// ---------------- Z축 층간 이동량 ----------------
// 모든 층의 간격은 Z_REVOLUTIONS_PER_FLOOR 상수 하나로 조절
// 1·2번 방: 기준 높이, 3·4번 방: 1층 간격, 5·6번 방: 2층 간격
const float Z_REVOLUTIONS_PER_FLOOR = 10.0f;

const float F_ROOM_Z_REVOLUTIONS[6] = {
  0.0f,                            // 1번 방
  0.0f,                            // 2번 방
  Z_REVOLUTIONS_PER_FLOOR,         // 3번 방
  Z_REVOLUTIONS_PER_FLOOR,         // 4번 방
  Z_REVOLUTIONS_PER_FLOOR * 2.0f,  // 5번 방
  Z_REVOLUTIONS_PER_FLOOR * 2.0f   // 6번 방
};

const float G_ROOM_Z_REVOLUTIONS[6] = {
  0.0f,                             // 1번 방 복귀 기준
  0.0f,                             // 2번 방 복귀 기준
  -Z_REVOLUTIONS_PER_FLOOR,         // 3번 방 복귀
  -Z_REVOLUTIONS_PER_FLOOR,         // 4번 방 복귀
  -Z_REVOLUTIONS_PER_FLOOR * 2.0f,  // 5번 방 복귀
  -Z_REVOLUTIONS_PER_FLOOR * 2.0f   // 6번 방 복귀
};

// ---------------- 스테퍼 상태 ----------------
struct StepperProfile {
  long currentPos = 0;
  long targetPos = 0;
  long totalSteps = 0;
  long stepCount = 0;
  float currentSpeed = MIN_SPEED;
  unsigned long lastStepMicros = 0;
  unsigned long stepIntervalMicros = 10000;
};

StepperProfile motorX;
StepperProfile motorZ;
StepperProfile motorY;

// ---------------- 동작 정의 ----------------
enum MotionAction : uint8_t {
  ACTION_A,
  ACTION_B,
  ACTION_C,
  ACTION_D,
  ACTION_E,
  ACTION_F,
  ACTION_G,
  ACTION_H
};

// 기존 적재 시퀀스
const MotionAction PUT_SEQUENCE[] = {
  ACTION_A,
  ACTION_B,
  ACTION_C,
  ACTION_D,
  ACTION_E,
  ACTION_F,
  ACTION_B,
  ACTION_A,
  ACTION_D,
  ACTION_C,
  ACTION_H,
  ACTION_G
};

// ---------------- 로봇 상태 ----------------
// 0 = 태그 인식 대기
// 1 = 모터 이동 중
// 9 = 동작 완료 후 대기

int sequenceState = 0;
const MotionAction *activeSequence = nullptr;
size_t activeSequenceLength = 0;
size_t activeActionIndex = 0;
uint8_t activeActionPhase = 0;
StepperProfile *activeMotor = nullptr;
int activeStepPin = -1;
int selectedRoom = -1;
int currentItemId = -1;

// false = 빈 방
// true  = 적재 완료
bool roomLoaded[6] = {
  false, false, false,
  false, false, false
};

unsigned long waitStartTime = 0;

// 태그 중복 인식 방지
int lastDetectedRoom = -1;
unsigned long lastDetectedTime = 0;
const unsigned long QR_COOLDOWN_MS = 1500;

// ---------------- 함수 선언 ----------------
void reportRoomStateConflict(int room);
void startRoom(int room);

bool startMotionAction(
  MotionAction action,
  uint8_t phase = 0
);

void moveTo(
  StepperProfile &motor,
  long target,
  int dirPin
);

void calculateNextStep(StepperProfile &motor);

bool runStepper(
  StepperProfile &motor,
  int stepPin
);

int readQrRoom();
bool i2cDevicePresent(uint8_t addr);

// =====================================================
// I2C 장치 확인
// =====================================================
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// =====================================================
// 모터 목표 위치 설정
// =====================================================
void moveTo(
  StepperProfile &motor,
  long target,
  int dirPin
) {
  motor.targetPos = target;
  motor.totalSteps = labs(target - motor.currentPos);
  motor.stepCount = 0;

  const float minSpeed =
    (&motor == &motorY)
      ? Y_MIN_SPEED
      : MIN_SPEED;

  motor.currentSpeed = minSpeed;
  motor.stepIntervalMicros =
    (unsigned long)(1000000.0f / minSpeed);

  motor.lastStepMicros = micros();

  digitalWrite(
    dirPin,
    target >= motor.currentPos ? HIGH : LOW
  );
}

// =====================================================
// A~H 동작 시작
// =====================================================
bool startMotionAction(
  MotionAction action,
  uint8_t phase
) {
  float revolutions = 0.0f;
  int dirPin = -1;

  activeMotor = nullptr;
  activeStepPin = -1;

  switch (action) {
    case ACTION_A:
      activeMotor = &motorY;
      activeStepPin = stepPinY;
      dirPin = dirPinY;
      revolutions = A_Y_FORWARD_REVOLUTIONS;
      break;

    case ACTION_B:
      activeMotor = &motorZ;
      activeStepPin = stepPinZ;
      dirPin = dirPinZ;
      revolutions = B_Z_FORWARD_REVOLUTIONS;
      break;

    case ACTION_C:
      activeMotor = &motorY;
      activeStepPin = stepPinY;
      dirPin = dirPinY;
      revolutions = -C_Y_REVERSE_REVOLUTIONS;
      break;

    case ACTION_D:
      activeMotor = &motorZ;
      activeStepPin = stepPinZ;
      dirPin = dirPinZ;
      revolutions = -D_Z_REVERSE_REVOLUTIONS;
      break;

    case ACTION_E:
      // 모든 방에서 공통 X축 이동 없음
      activeMotor = &motorX;
      activeStepPin = stepPinX;
      dirPin = dirPinX;
      revolutions = E_X_FORWARD_REVOLUTIONS;
      break;

    case ACTION_F:
      if (phase == 0) {
        // 방 2, 4, 6만 X축 이동
        activeMotor = &motorX;
        activeStepPin = stepPinX;
        dirPin = dirPinX;
        revolutions = F_ROOM_X_REVOLUTIONS[selectedRoom];
      } else {
        // 방의 층에 따라 Z축 이동
        activeMotor = &motorZ;
        activeStepPin = stepPinZ;
        dirPin = dirPinZ;
        revolutions = F_ROOM_Z_REVOLUTIONS[selectedRoom];
      }
      break;

    case ACTION_G:
      if (phase == 0) {
        // Z축 복귀
        activeMotor = &motorZ;
        activeStepPin = stepPinZ;
        dirPin = dirPinZ;
        revolutions = G_ROOM_Z_REVOLUTIONS[selectedRoom];
      } else {
        // 방 2, 4, 6만 X축 복귀
        activeMotor = &motorX;
        activeStepPin = stepPinX;
        dirPin = dirPinX;
        revolutions = G_ROOM_X_REVOLUTIONS[selectedRoom];
      }
      break;

    case ACTION_H:
      // 공통 X축 복귀 없음
      activeMotor = &motorX;
      activeStepPin = stepPinX;
      dirPin = dirPinX;
      revolutions = -H_X_REVERSE_REVOLUTIONS;
      break;
  }

  if (activeMotor == nullptr) {
    return false;
  }

  long deltaSteps =
    lroundf(revolutions * STEPS_PER_REVOLUTION);

  moveTo(
    *activeMotor,
    activeMotor->currentPos + deltaSteps,
    dirPin
  );

  Serial.print(F("Action "));
  Serial.print((char)('A' + (int)action));
  Serial.print(F(" | Phase "));
  Serial.print(phase + 1);
  Serial.print(F(" | Revolutions "));
  Serial.println(revolutions, 3);

  return true;
}

// =====================================================
// 가속 / 감속 계산
// =====================================================
void calculateNextStep(StepperProfile &motor) {
  motor.stepCount++;

  long remaining =
    motor.totalSteps - motor.stepCount;

  if (remaining <= 0) {
    return;
  }

  const bool isYMotor = (&motor == &motorY);

  const float maxSpeed =
    isYMotor ? Y_MAX_SPEED : MAX_SPEED;

  const float acceleration =
    isYMotor ? Y_ACCELERATION : ACCELERATION;

  const float minSpeed =
    isYMotor ? Y_MIN_SPEED : MIN_SPEED;

  float accelSpeed =
    motor.currentSpeed +
    acceleration *
    motor.stepIntervalMicros / 1000000.0f;

  float decelSpeed =
    sqrtf(2.0f * acceleration * remaining);

  motor.currentSpeed =
    fminf(accelSpeed, maxSpeed);

  motor.currentSpeed =
    fminf(motor.currentSpeed, decelSpeed);

  motor.currentSpeed =
    fmaxf(motor.currentSpeed, minSpeed);

  motor.stepIntervalMicros =
    (unsigned long)(1000000.0f / motor.currentSpeed);
}

// =====================================================
// 스테퍼 모터 구동
// =====================================================
bool runStepper(
  StepperProfile &motor,
  int stepPin
) {
  if (motor.currentPos == motor.targetPos) {
    return true;
  }

  unsigned long now = micros();

  if (
    now - motor.lastStepMicros <
    motor.stepIntervalMicros
  ) {
    return false;
  }

  motor.lastStepMicros = now;

  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);

  motor.currentPos +=
    (motor.targetPos > motor.currentPos)
      ? 1
      : -1;

  calculateNextStep(motor);

  return motor.currentPos == motor.targetPos;
}

// =====================================================
// HuskyLens 태그 읽기
// ID 1은 무시하고 ID 2~6만 허용
// =====================================================
int readQrRoom() {
  if (!huskylens.request()) {
    return -1;
  }

  if (!huskylens.available()) {
    return -1;
  }

  HUSKYLENSResult result = huskylens.read();

  if (result.command != COMMAND_RETURN_BLOCK) {
    return -1;
  }

  currentItemId = result.ID;

  // ID 1은 무시
  if (result.ID == 1) {
    return -1;
  }

  // ID 2~6만 방 번호로 사용
  if (result.ID >= 2 && result.ID <= 6) {
    Serial.print(F("Detected Room Tag ID: "));
    Serial.println(result.ID);

    // ID 2 -> 인덱스 1 (2번 방)
    // ID 6 -> 인덱스 5 (6번 방)
    return result.ID - 1;
  }

  // 그 외 태그 ID 무시
  return -1;
}

// =====================================================
// 이미 적재된 방의 재적재 거부
// =====================================================
void reportRoomStateConflict(int room) {
  Serial.println(F("=============================="));
  Serial.print(F("PUT REJECTED: ROOM "));
  Serial.println(room + 1);

  Serial.println(
    F("Reason: This room is already loaded.")
  );

  Serial.println(F("Motors will not move."));
  Serial.println(F("=============================="));
}

// =====================================================
// 방 선택 및 적재 동작 시작
// =====================================================
void startRoom(int room) {
  if (room < 1 || room > 5) {
    Serial.println(F("Invalid room index."));
    return;
  }

  if (roomLoaded[room]) {
    reportRoomStateConflict(room);
    return;
  }

  selectedRoom = room;
  activeSequence = PUT_SEQUENCE;

  activeSequenceLength =
    sizeof(PUT_SEQUENCE) /
    sizeof(PUT_SEQUENCE[0]);

  activeActionIndex = 0;
  activeActionPhase = 0;

  if (
    !startMotionAction(
      activeSequence[activeActionIndex],
      activeActionPhase
    )
  ) {
    Serial.println(
      F("ERROR: Could not start motion sequence.")
    );

    activeSequence = nullptr;
    activeSequenceLength = 0;
    activeMotor = nullptr;
    activeStepPin = -1;
    sequenceState = 0;

    return;
  }

  sequenceState = 1;

  Serial.println(F("=============================="));
  Serial.print(F("START PUT SEQUENCE: ROOM "));
  Serial.println(room + 1);
  Serial.println(F("=============================="));
}

// =====================================================
// 초기 설정
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(dirPinZ, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);

  digitalWrite(stepPinX, LOW);
  digitalWrite(stepPinZ, LOW);
  digitalWrite(stepPinY, LOW);

  Wire.begin();
  Wire.setClock(I2C_CLOCK);

  delay(200);

  Serial.println();
  Serial.println(F("Arduino Mega Logistics Robot"));
  Serial.println(F("I2C device check"));

  bool huskyFound = i2cDevicePresent(0x32);

  Serial.print(F("HuskyLens address 0x32: "));
  Serial.println(
    huskyFound ? F("FOUND") : F("NOT FOUND")
  );

  if (!huskyFound) {
    Serial.println(
      F("Check SDA=20, SCL=21, power and GND.")
    );
  }

  bool huskyReady = false;

  for (int attempt = 0; attempt < 10; attempt++) {
    Serial.print(F("HuskyLens initialization: "));
    Serial.println(attempt + 1);

    if (huskylens.begin(Wire)) {
      huskyReady = true;
      break;
    }

    delay(500);
  }

  if (!huskyReady) {
    Serial.println(F("HUSKYLENS INITIALIZATION FAILED"));
    Serial.println(
      F("Set HuskyLens Protocol Type to I2C.")
    );

    while (true) {
      delay(1000);
    }
  }

  huskylens.writeAlgorithm(
    ALGORITHM_TAG_RECOGNITION
  );

  Serial.println(F("HuskyLens ready."));
  Serial.println(F("Algorithm: Tag Recognition"));
  Serial.println(F("Accepted tag IDs: 2~6"));
  Serial.println(F("Tag ID 1 is ignored."));
  Serial.println(F("All rooms initially marked EMPTY."));
  Serial.println(F("Waiting for room tag..."));

  Serial.println();
}

// =====================================================
// 메인 루프
// =====================================================
void loop() {
  unsigned long now = millis();

  switch (sequenceState) {
    case 0: {
      int room = readQrRoom();

      // ID 1 및 유효하지 않은 태그는 무시
      if (room < 0) {
        break;
      }

      if (
        room == lastDetectedRoom &&
        now - lastDetectedTime < QR_COOLDOWN_MS
      ) {
        break;
      }

      lastDetectedRoom = room;
      lastDetectedTime = now;

      Serial.print(F("Room selected: "));
      Serial.println(room + 1);

      if (roomLoaded[room]) {
        reportRoomStateConflict(room);
        break;
      }

      startRoom(room);
      break;
    }

    case 1: {
      if (
        activeMotor == nullptr ||
        activeStepPin < 0 ||
        activeSequence == nullptr
      ) {
        Serial.println(
          F("ERROR: Invalid motion state.")
        );

        activeSequence = nullptr;
        activeSequenceLength = 0;
        activeMotor = nullptr;
        activeStepPin = -1;
        sequenceState = 0;

        break;
      }

      if (
        runStepper(
          *activeMotor,
          activeStepPin
        )
      ) {
        MotionAction action =
          activeSequence[activeActionIndex];

        // F/G는 2단계 이동
        if (
          (action == ACTION_F ||
           action == ACTION_G) &&
          activeActionPhase == 0
        ) {
          activeActionPhase = 1;

          if (
            !startMotionAction(
              action,
              activeActionPhase
            )
          ) {
            Serial.println(
              F("ERROR: Failed to start second phase.")
            );

            activeSequence = nullptr;
            activeSequenceLength = 0;
            activeMotor = nullptr;
            activeStepPin = -1;
            sequenceState = 0;
          }

          break;
        }

        activeActionPhase = 0;
        activeActionIndex++;

        if (
          activeActionIndex <
          activeSequenceLength
        ) {
          if (
            !startMotionAction(
              activeSequence[activeActionIndex],
              activeActionPhase
            )
          ) {
            Serial.println(
              F("ERROR: Failed to start next action.")
            );

            activeSequence = nullptr;
            activeSequenceLength = 0;
            activeMotor = nullptr;
            activeStepPin = -1;
            sequenceState = 0;
          }
        } else {
          roomLoaded[selectedRoom] = true;

          Serial.println();
          Serial.println(F("=============================="));
          Serial.print(F("PUT COMPLETE: ROOM "));
          Serial.println(selectedRoom + 1);

          Serial.println(
            F("Room state changed to LOADED.")
          );

          Serial.println(F("=============================="));
          Serial.println();

          activeSequence = nullptr;
          activeSequenceLength = 0;
          activeMotor = nullptr;
          activeStepPin = -1;

          waitStartTime = millis();
          sequenceState = 9;
        }
      }

      break;
    }

    case 9: {
      if (now - waitStartTime >= 1500) {
        Serial.println(F("Waiting for next room tag..."));
        sequenceState = 0;
      }

      break;
    }
  }
}
