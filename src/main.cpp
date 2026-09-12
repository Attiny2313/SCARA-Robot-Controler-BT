#include <Arduino.h>
#include <Bluepad32.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ============================================================
// PINY
// ============================================================
constexpr uint8_t PIN_LED_BT       = 4;
constexpr uint8_t PIN_LED_ERROR    = 16;
constexpr uint8_t PIN_LED_STATUS   = 17;

constexpr uint8_t PIN_STEP_ARM1    = 13;
constexpr uint8_t PIN_DIR_ARM1     = 26;
constexpr uint8_t PIN_STEP_ARM2    = 23;
constexpr uint8_t PIN_DIR_ARM2     = 14;
constexpr uint8_t PIN_STEP_Z       = 18;
constexpr uint8_t PIN_DIR_Z        = 19;

constexpr uint8_t PIN_SERVO_ROTATE = 27;
constexpr uint8_t PIN_SERVO_GRIP   = 32;

constexpr uint8_t PIN_BUTTON_TEACH = 25;
constexpr uint8_t PIN_BUTTON_MODE  = 33;
constexpr uint8_t PIN_ESTOP        = 39;

// Zostawione sprzętowo na przyszłość. Homing w tym projekcie jest ręczny.
constexpr uint8_t PIN_ENDSTOP_Z    = 34;
constexpr uint8_t PIN_ENDSTOP_ARM1 = 35;
constexpr uint8_t PIN_ENDSTOP_ARM2 = 36;

// ============================================================
// KONFIGURACJA MECHANIKI
// ============================================================
constexpr float ARM_LENGTH_1_MM = 91.61f;
constexpr float ARM_LENGTH_2_MM = 97.528f;

constexpr float MOTOR_STEPS_REV = 200.0f;
constexpr float MICROSTEPS      = 16.0f;

constexpr float GEAR_ARM1 = 3.875f;
constexpr float GEAR_ARM2 = 7.280f;
constexpr float Z_SCREW_LEAD_MM = 2.0f;
constexpr float GEAR_Z          = 1.0f;

constexpr int8_t SIGN_ARM1 = 1;
constexpr int8_t SIGN_ARM2 = 1;
constexpr int8_t SIGN_Z    = 1;

constexpr float STEPS_PER_DEG_ARM1 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM1 / 360.0f;
constexpr float STEPS_PER_DEG_ARM2 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM2 / 360.0f;
constexpr float STEPS_PER_MM_Z =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_Z / Z_SCREW_LEAD_MM;

// ============================================================
// RĘCZNY HOME
// ============================================================
// Przed włączeniem/resetem robot musi zostać ręcznie ustawiony w HOME.
// Program przyjmuje poniższe współrzędne jako aktualną pozycję po starcie.
constexpr float HOME_ARM1_DEG = 115.0f;
constexpr float HOME_ARM2_DEG = -147.0f;
constexpr float HOME_Z_MM     = 0.0f;
constexpr float HOME_TOOL_ROTATE_DEG = 90.0f;
constexpr float HOME_GRIP_DEG        = 30.0f;

// ============================================================
// OGRANICZENIA PROGRAMOWE
// ============================================================
constexpr float ARM1_MIN_DEG = -115.0f;
constexpr float ARM1_MAX_DEG =  115.0f;
constexpr float ARM2_MIN_DEG = -147.0f;
constexpr float ARM2_MAX_DEG =  147.0f;
constexpr float Z_MIN_MM = 0.0f;
constexpr float Z_MAX_MM = 300.0f;
constexpr float TOOL_ROTATE_MIN_DEG = 0.0f;
constexpr float TOOL_ROTATE_MAX_DEG = 180.0f;
constexpr float GRIP_MIN_DEG = 15.0f;
constexpr float GRIP_MAX_DEG = 95.0f;

// ============================================================
// PRĘDKOŚCI
// ============================================================
// Ramię 1 pracowało poprawnie, więc zostaje przy dotychczasowej prędkości.
// Ramię 2 ma większe przełożenie i zostało mocno spowolnione.
constexpr float ARM1_JOG_SPEED_DEG_S = 25.0f;
constexpr float ARM2_JOG_SPEED_DEG_S = 10.0f;

// Z: normalna praca 0.75 mm/s, absolutnie nigdy powyżej 1.0 mm/s.
constexpr float Z_JOG_SPEED_MM_S     = 0.75f;
constexpr float TOOL_XY_SPEED_MM_S   = 20.0f;

// AUTO ma osobne limity osi. Dzięki temu ramię 2 i Z nie są zmuszane
// do prędkości ramienia 1, a PTP nadal kończy ruch osi możliwie jednocześnie.
constexpr float AUTO_ARM1_SPEED_DEG_S = 20.0f;
constexpr float AUTO_ARM2_SPEED_DEG_S = 8.0f;
constexpr float AUTO_Z_SPEED_MM_S     = 0.75f;

constexpr float TOOL_ROTATE_SPEED_DEG_S = 35.0f;
constexpr float GRIP_SPEED_DEG_S        = 45.0f;

constexpr int SERVO_ROTATE_MIN_US = 500;
constexpr int SERVO_ROTATE_MAX_US = 2500;
constexpr int SERVO_GRIP_MIN_US   = 500;
constexpr int SERVO_GRIP_MAX_US   = 2500;
constexpr float SERVO_FILTER_ALPHA = 0.20f;

// Twarde limity częstotliwości STEP - dodatkowe zabezpieczenie niezależne
// od prędkości zadawanej przez JOINT / TOOL / AUTO.
constexpr uint32_t ARM1_MAX_STEP_HZ = 10000;
constexpr uint32_t ARM2_MAX_STEP_HZ = 3000;
// Przy 1600 STEP/mm: 1600 Hz = dokładnie 1.0 mm/s.
constexpr uint32_t Z_MAX_STEP_HZ    = 1600;

constexpr uint32_t ARM1_ACCEL = 12000;
constexpr uint32_t ARM2_ACCEL = 5000;
// Bardzo łagodny start Z: 300 STEP/s^2 = 0.1875 mm/s^2 przy 1600 STEP/mm.
constexpr uint32_t Z_ACCEL    = 300;

// ============================================================
// CZASY / AUTO
// ============================================================
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t SERVO_PERIOD_MS = 20;
constexpr uint32_t GAMEPAD_TIMEOUT_MS = 500;
constexpr uint32_t GAMEPAD_ARM_NEUTRAL_MS = 300;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t TEACH_LONG_PRESS_MS = 1800;
constexpr uint32_t DEFAULT_POINT_DWELL_MS = 400;
constexpr int16_t JOYSTICK_DEADZONE = 90;
constexpr float MAX_CONTROL_DT_S = 0.05f;
constexpr uint8_t MAX_PROGRAM_POINTS = 20;
constexpr bool AUTO_LOOP_PROGRAM = true;

// ============================================================
// STRUKTURY
// ============================================================
enum class ControlMode : uint8_t {
    JOINT,
    TOOL
};

enum class OperatingMode : uint8_t {
    MANUAL,
    AUTO
};

enum class AutoState : uint8_t {
    IDLE,
    MOVE,
    DWELL,
    PAUSED,
    FINISHED,
    FAULT
};

struct JointPosition {
    float arm1Deg;
    float arm2Deg;
    float zMm;
};

struct CartesianPosition {
    float xMm;
    float yMm;
    float zMm;
};

struct ProgramPoint {
    JointPosition joints;
    float toolRotateDeg;
    float gripperDeg;
    uint32_t dwellMs;
};

struct RobotState {
    JointPosition joints;       // pozycja zadana
    JointPosition actualJoints; // pozycja z liczników kroków
    CartesianPosition tcp;
    CartesianPosition actualTcp;

    float toolRotateDeg;
    float gripperDeg;

    ControlMode controlMode;
    OperatingMode operatingMode;
    AutoState autoState;

    bool estopLatched;
    bool controllerConnected;
    bool gamepadArmed;
};

// ============================================================
// OBIEKTY / STAN
// ============================================================
FastAccelStepperEngine stepperEngine;
FastAccelStepper* stepperArm1 = nullptr;
FastAccelStepper* stepperArm2 = nullptr;
FastAccelStepper* stepperZ    = nullptr;

Servo servoRotate;
Servo servoGrip;
Preferences preferences;
ControllerPtr controllers[BP32_MAX_GAMEPADS];
RobotState robot;

ProgramPoint programPoints[MAX_PROGRAM_POINTS];
uint8_t programPointCount = 0;
uint8_t currentProgramPoint = 0;
bool autoMotionStarted = false;
uint32_t autoDwellStartedMs = 0;

uint32_t lastControlMs = 0;
uint32_t lastServoUpdateMs = 0;
uint32_t lastGamepadPacketMs = 0;
uint32_t gamepadNeutralSinceMs = 0;
uint32_t lastModeButtonEdgeMs = 0;
uint32_t lastTeachButtonEdgeMs = 0;
uint32_t teachPressStartedMs = 0;

float servoRotateFilteredDeg = HOME_TOOL_ROTATE_DEG;
float servoGripFilteredDeg = HOME_GRIP_DEG;

bool previousModeButton = HIGH;
bool previousTeachButton = HIGH;
bool previousPadStart = false;

// ============================================================
// POMOCNICZE
// ============================================================
float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

uint32_t clampStepHz(float hz, uint32_t maximumHz) {
    if (hz < 1.0f) return 1;
    if (hz > static_cast<float>(maximumHz)) return maximumHz;
    return static_cast<uint32_t>(lroundf(hz));
}

float degToRad(float degrees) {
    return degrees * PI / 180.0f;
}

float radToDeg(float radians) {
    return radians * 180.0f / PI;
}

float normalizeJoystick(int32_t value) {
    if (abs(value) < JOYSTICK_DEADZONE) return 0.0f;
    constexpr float maximumAxisValue = 512.0f;
    return clampFloat(static_cast<float>(value) / maximumAxisValue, -1.0f, 1.0f);
}

int32_t arm1DegreesToSteps(float angleDeg) {
    return lroundf(angleDeg * STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

int32_t arm2DegreesToSteps(float angleDeg) {
    return lroundf(angleDeg * STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

int32_t zMillimetersToSteps(float zMm) {
    return lroundf(zMm * STEPS_PER_MM_Z * SIGN_Z);
}

float arm1StepsToDegrees(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

float arm2StepsToDegrees(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

float zStepsToMillimeters(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_MM_Z * SIGN_Z);
}

// ============================================================
// KINEMATYKA
// ============================================================
CartesianPosition forwardKinematics(const JointPosition& joints) {
    const float theta1 = degToRad(joints.arm1Deg);
    const float theta2 = degToRad(joints.arm2Deg);

    CartesianPosition result;
    result.xMm = ARM_LENGTH_1_MM * cosf(theta1) +
                 ARM_LENGTH_2_MM * cosf(theta1 + theta2);
    result.yMm = ARM_LENGTH_1_MM * sinf(theta1) +
                 ARM_LENGTH_2_MM * sinf(theta1 + theta2);
    result.zMm = joints.zMm;
    return result;
}

bool inverseKinematics(float xMm, float yMm, bool elbowUp, JointPosition& result) {
    const float radiusSquared = xMm * xMm + yMm * yMm;
    float cosTheta2 =
        (radiusSquared - ARM_LENGTH_1_MM * ARM_LENGTH_1_MM -
         ARM_LENGTH_2_MM * ARM_LENGTH_2_MM) /
        (2.0f * ARM_LENGTH_1_MM * ARM_LENGTH_2_MM);

    if (cosTheta2 < -1.0f || cosTheta2 > 1.0f) return false;

    cosTheta2 = clampFloat(cosTheta2, -1.0f, 1.0f);
    float sinTheta2 = sqrtf(1.0f - cosTheta2 * cosTheta2);
    if (!elbowUp) sinTheta2 = -sinTheta2;

    const float theta2 = atan2f(sinTheta2, cosTheta2);
    const float theta1 = atan2f(yMm, xMm) -
        atan2f(ARM_LENGTH_2_MM * sinTheta2,
               ARM_LENGTH_1_MM + ARM_LENGTH_2_MM * cosTheta2);

    result.arm1Deg = radToDeg(theta1);
    result.arm2Deg = radToDeg(theta2);
    result.zMm = robot.joints.zMm;

    return !(result.arm1Deg < ARM1_MIN_DEG || result.arm1Deg > ARM1_MAX_DEG ||
             result.arm2Deg < ARM2_MIN_DEG || result.arm2Deg > ARM2_MAX_DEG);
}

// ============================================================
// POZYCJA / RUCH
// ============================================================
void updateActualRobotPosition() {
    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return;

    robot.actualJoints.arm1Deg = arm1StepsToDegrees(stepperArm1->getCurrentPosition());
    robot.actualJoints.arm2Deg = arm2StepsToDegrees(stepperArm2->getCurrentPosition());
    robot.actualJoints.zMm = zStepsToMillimeters(stepperZ->getCurrentPosition());
    robot.actualTcp = forwardKinematics(robot.actualJoints);
}

bool axesRunning() {
    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return false;
    return stepperArm1->isRunning() || stepperArm2->isRunning() || stepperZ->isRunning();
}

bool motionFinished() {
    return !axesRunning();
}

JointPosition clampJointPosition(const JointPosition& target) {
    return {
        clampFloat(target.arm1Deg, ARM1_MIN_DEG, ARM1_MAX_DEG),
        clampFloat(target.arm2Deg, ARM2_MIN_DEG, ARM2_MAX_DEG),
        clampFloat(target.zMm, Z_MIN_MM, Z_MAX_MM)
    };
}

void emergencyStopMotion() {
    if (stepperArm1 != nullptr) stepperArm1->forceStop();
    if (stepperArm2 != nullptr) stepperArm2->forceStop();
    if (stepperZ != nullptr) stepperZ->forceStop();
}

void commandManualJointPosition(const JointPosition& target, float deltaTimeSeconds) {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL) return;

    const JointPosition clamped = clampJointPosition(target);

    if (deltaTimeSeconds > 0.0f) {
        const float arm1Delta = fabsf(clamped.arm1Deg - robot.joints.arm1Deg);
        const float arm2Delta = fabsf(clamped.arm2Deg - robot.joints.arm2Deg);
        const float zDelta = fabsf(clamped.zMm - robot.joints.zMm);

        const uint32_t arm1ManualMaxHz = clampStepHz(
            ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1, ARM1_MAX_STEP_HZ);
        const uint32_t arm2ManualMaxHz = clampStepHz(
            ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2, ARM2_MAX_STEP_HZ);
        const uint32_t zManualMaxHz = clampStepHz(
            Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z, Z_MAX_STEP_HZ);

        if (arm1Delta > 0.0001f) {
            stepperArm1->setSpeedInHz(clampStepHz(
                arm1Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM1, arm1ManualMaxHz));
        }
        if (arm2Delta > 0.0001f) {
            stepperArm2->setSpeedInHz(clampStepHz(
                arm2Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM2, arm2ManualMaxHz));
        }
        if (zDelta > 0.0001f) {
            stepperZ->setSpeedInHz(clampStepHz(
                zDelta / deltaTimeSeconds * STEPS_PER_MM_Z, zManualMaxHz));
        }
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

void commandAutoJointPosition(const JointPosition& target) {
    if (robot.estopLatched || robot.operatingMode != OperatingMode::AUTO) return;

    updateActualRobotPosition();
    const JointPosition clamped = clampJointPosition(target);

    const float d1 = fabsf(clamped.arm1Deg - robot.actualJoints.arm1Deg);
    const float d2 = fabsf(clamped.arm2Deg - robot.actualJoints.arm2Deg);
    const float dz = fabsf(clamped.zMm - robot.actualJoints.zMm);

    const float t1 = d1 / AUTO_ARM1_SPEED_DEG_S;
    const float t2 = d2 / AUTO_ARM2_SPEED_DEG_S;
    const float tz = dz / AUTO_Z_SPEED_MM_S;
    const float moveTime = fmaxf(0.01f, fmaxf(t1, fmaxf(t2, tz)));

    if (d1 > 0.001f) {
        stepperArm1->setSpeedInHz(clampStepHz(
            d1 / moveTime * STEPS_PER_DEG_ARM1, ARM1_MAX_STEP_HZ));
    }
    if (d2 > 0.001f) {
        stepperArm2->setSpeedInHz(clampStepHz(
            d2 / moveTime * STEPS_PER_DEG_ARM2, ARM2_MAX_STEP_HZ));
    }
    if (dz > 0.001f) {
        stepperZ->setSpeedInHz(clampStepHz(
            dz / moveTime * STEPS_PER_MM_Z, Z_MAX_STEP_HZ));
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

// ============================================================
// PROGRAM TEACH / NVS
// ============================================================
void saveProgram() {
    preferences.putUChar("count", programPointCount);
    if (programPointCount > 0) {
        preferences.putBytes("points", programPoints,
                             sizeof(ProgramPoint) * programPointCount);
    } else {
        preferences.remove("points");
    }
}

void loadProgram() {
    programPointCount = preferences.getUChar("count", 0);
    if (programPointCount > MAX_PROGRAM_POINTS) {
        programPointCount = 0;
        saveProgram();
        return;
    }

    if (programPointCount > 0) {
        const size_t expected = sizeof(ProgramPoint) * programPointCount;
        if (preferences.getBytesLength("points") != expected ||
            preferences.getBytes("points", programPoints, expected) != expected) {
            programPointCount = 0;
            saveProgram();
        }
    }
}

void clearProgram() {
    if (robot.operatingMode != OperatingMode::MANUAL || axesRunning()) return;
    programPointCount = 0;
    currentProgramPoint = 0;
    saveProgram();
    Serial.println("TEACH: program wyczyszczony");
}

void teachCurrentPoint() {
    if (robot.estopLatched || robot.operatingMode != OperatingMode::MANUAL) return;
    if (axesRunning()) {
        Serial.println("TEACH: zatrzymaj osie przed zapisaniem punktu");
        return;
    }
    if (programPointCount >= MAX_PROGRAM_POINTS) {
        Serial.println("TEACH: brak miejsca na kolejny punkt");
        return;
    }

    updateActualRobotPosition();
    ProgramPoint& p = programPoints[programPointCount];
    p.joints = robot.actualJoints;
    p.toolRotateDeg = robot.toolRotateDeg;
    p.gripperDeg = robot.gripperDeg;
    p.dwellMs = DEFAULT_POINT_DWELL_MS;

    programPointCount++;
    saveProgram();

    Serial.printf("TEACH: zapisano P%02u  A1=%.2f A2=%.2f Z=%.2f ROT=%.1f GRIP=%.1f\n",
                  programPointCount,
                  p.joints.arm1Deg, p.joints.arm2Deg, p.joints.zMm,
                  p.toolRotateDeg, p.gripperDeg);
}

// ============================================================
// AUTO
// ============================================================
void setAutoFault(const char* reason) {
    emergencyStopMotion();
    robot.autoState = AutoState::FAULT;
    autoMotionStarted = false;
    digitalWrite(PIN_LED_ERROR, HIGH);
    if (reason != nullptr) Serial.println(reason);
}

void pauseAuto(const char* reason) {
    if (robot.operatingMode != OperatingMode::AUTO) return;
    if (robot.autoState == AutoState::IDLE ||
        robot.autoState == AutoState::FINISHED ||
        robot.autoState == AutoState::FAULT) return;

    emergencyStopMotion();
    updateActualRobotPosition();
    robot.joints = robot.actualJoints;
    robot.tcp = robot.actualTcp;
    robot.autoState = AutoState::PAUSED;
    autoMotionStarted = false;
    if (reason != nullptr) Serial.println(reason);
}

void startAuto() {
    if (robot.estopLatched || robot.operatingMode != OperatingMode::AUTO) return;
    if (programPointCount == 0) {
        Serial.println("AUTO: brak zapisanych punktów");
        return;
    }

    currentProgramPoint = 0;
    autoMotionStarted = false;
    robot.autoState = AutoState::MOVE;
    digitalWrite(PIN_LED_ERROR, LOW);
    Serial.printf("AUTO START: %u punktów\n", programPointCount);
}

void toggleAutoRunPause() {
    if (robot.operatingMode != OperatingMode::AUTO || robot.estopLatched) return;

    switch (robot.autoState) {
        case AutoState::IDLE:
        case AutoState::FINISHED:
            startAuto();
            break;

        case AutoState::MOVE:
        case AutoState::DWELL:
            pauseAuto("AUTO PAUSE");
            break;

        case AutoState::PAUSED:
            robot.autoState = AutoState::MOVE;
            autoMotionStarted = false;
            Serial.printf("AUTO RESUME: P%02u\n", currentProgramPoint + 1);
            break;

        case AutoState::FAULT:
            Serial.println("AUTO: FAULT - wymagany restart po E-STOP");
            break;
    }
}

void startCurrentProgramPoint() {
    if (currentProgramPoint >= programPointCount) {
        setAutoFault("AUTO: błędny indeks punktu");
        return;
    }

    const ProgramPoint& p = programPoints[currentProgramPoint];
    robot.toolRotateDeg = clampFloat(p.toolRotateDeg,
                                    TOOL_ROTATE_MIN_DEG, TOOL_ROTATE_MAX_DEG);
    robot.gripperDeg = clampFloat(p.gripperDeg,
                                 GRIP_MIN_DEG, GRIP_MAX_DEG);
    commandAutoJointPosition(p.joints);
    autoMotionStarted = true;

    Serial.printf("AUTO -> P%02u/%02u\n",
                  currentProgramPoint + 1, programPointCount);
}

void processAuto() {
    if (robot.operatingMode != OperatingMode::AUTO || robot.estopLatched) return;

    switch (robot.autoState) {
        case AutoState::IDLE:
        case AutoState::PAUSED:
        case AutoState::FINISHED:
        case AutoState::FAULT:
            return;

        case AutoState::MOVE:
            if (!autoMotionStarted) {
                startCurrentProgramPoint();
                if (robot.autoState == AutoState::FAULT) return;
            }

            if (autoMotionStarted && motionFinished()) {
                autoMotionStarted = false;
                autoDwellStartedMs = millis();
                robot.autoState = AutoState::DWELL;
            }
            break;

        case AutoState::DWELL: {
            const uint32_t dwell = programPoints[currentProgramPoint].dwellMs;
            if (millis() - autoDwellStartedMs < dwell) return;

            currentProgramPoint++;
            if (currentProgramPoint >= programPointCount) {
                if (AUTO_LOOP_PROGRAM) {
                    currentProgramPoint = 0;
                    Serial.println("AUTO: kolejny cykl");
                } else {
                    robot.autoState = AutoState::FINISHED;
                    Serial.println("AUTO: program zakończony");
                    return;
                }
            }

            robot.autoState = AutoState::MOVE;
            autoMotionStarted = false;
            break;
        }
    }
}

// ============================================================
// E-STOP
// ============================================================
void updateEmergencyStop() {
    const bool estopActive = digitalRead(PIN_ESTOP) == HIGH;

    if (estopActive && !robot.estopLatched) {
        robot.estopLatched = true;
        robot.gamepadArmed = false;
        emergencyStopMotion();
        robot.autoState = AutoState::FAULT;
        digitalWrite(PIN_LED_ERROR, HIGH);
        digitalWrite(PIN_LED_STATUS, LOW);
        Serial.println("E-STOP AKTYWNY - wymagany restart sterownika");
    }
}

// ============================================================
// TRYBY / PANEL
// ============================================================
void toggleControlMode() {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL) return;

    robot.controlMode =
        (robot.controlMode == ControlMode::JOINT) ? ControlMode::TOOL : ControlMode::JOINT;

    Serial.println(robot.controlMode == ControlMode::TOOL ? "MANUAL: TOOL" : "MANUAL: JOINT");
}

void toggleOperatingMode() {
    if (robot.estopLatched) return;
    if (axesRunning()) {
        Serial.println("MODE: zatrzymaj osie przed zmianą MANUAL/AUTO");
        return;
    }

    if (robot.operatingMode == OperatingMode::MANUAL) {
        robot.operatingMode = OperatingMode::AUTO;
        robot.autoState = AutoState::IDLE;
        robot.gamepadArmed = false;
        previousPadStart = false;
        Serial.printf("Tryb pracy: AUTO (%u punktów). START = uruchom/pauza\n",
                      programPointCount);
    } else {
        emergencyStopMotion();
        robot.operatingMode = OperatingMode::MANUAL;
        robot.autoState = AutoState::IDLE;
        autoMotionStarted = false;
        robot.gamepadArmed = false;
        gamepadNeutralSinceMs = 0;
        previousPadStart = false;
        updateActualRobotPosition();
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
        Serial.println("Tryb pracy: MANUAL - oczekiwanie na neutralny pad");
    }

    digitalWrite(PIN_LED_STATUS,
                 robot.operatingMode == OperatingMode::AUTO ? HIGH : LOW);
}

void printCurrentPosition() {
    updateActualRobotPosition();

    Serial.println();
    Serial.println("----- STAN ROBOTA -----");
    Serial.printf("A1 ACT/CMD: %.2f / %.2f deg\n",
                  robot.actualJoints.arm1Deg, robot.joints.arm1Deg);
    Serial.printf("A2 ACT/CMD: %.2f / %.2f deg\n",
                  robot.actualJoints.arm2Deg, robot.joints.arm2Deg);
    Serial.printf("Z  ACT/CMD: %.2f / %.2f mm\n",
                  robot.actualJoints.zMm, robot.joints.zMm);
    Serial.printf("TCP ACT X/Y: %.2f / %.2f mm\n",
                  robot.actualTcp.xMm, robot.actualTcp.yMm);
    Serial.printf("ROT: %.1f  GRIP: %.1f\n",
                  robot.toolRotateDeg, robot.gripperDeg);
    Serial.printf("WORK: %s  MANUAL: %s\n",
                  robot.operatingMode == OperatingMode::AUTO ? "AUTO" : "MANUAL",
                  robot.controlMode == ControlMode::TOOL ? "TOOL" : "JOINT");
    Serial.printf("TEACH points: %u/%u\n", programPointCount, MAX_PROGRAM_POINTS);
    Serial.printf("Pad: %s\n", robot.gamepadArmed ? "ARMED" : "SAFE");
    Serial.println("-----------------------");
}

void updatePanelButtons() {
    const uint32_t now = millis();
    const bool modeButton = digitalRead(PIN_BUTTON_MODE);
    const bool teachButton = digitalRead(PIN_BUTTON_TEACH);

    if (previousModeButton == HIGH && modeButton == LOW &&
        now - lastModeButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastModeButtonEdgeMs = now;
        toggleOperatingMode();
    }

    if (previousTeachButton == HIGH && teachButton == LOW &&
        now - lastTeachButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastTeachButtonEdgeMs = now;
        teachPressStartedMs = now;
    }

    if (previousTeachButton == LOW && teachButton == HIGH &&
        now - lastTeachButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastTeachButtonEdgeMs = now;
        const uint32_t heldMs = now - teachPressStartedMs;
        if (heldMs >= TEACH_LONG_PRESS_MS) {
            clearProgram();
        } else {
            teachCurrentPoint();
        }
        teachPressStartedMs = 0;
    }

    previousModeButton = modeButton;
    previousTeachButton = teachButton;
}

// ============================================================
// BLUEPAD32
// ============================================================
ControllerPtr getActiveController() {
    for (ControllerPtr controller : controllers) {
        if (controller != nullptr && controller->isConnected() && controller->isGamepad()) {
            return controller;
        }
    }
    return nullptr;
}

void updateControllerConnectedFlag() {
    robot.controllerConnected = getActiveController() != nullptr;
    digitalWrite(PIN_LED_BT, robot.controllerConnected ? HIGH : LOW);
}

void onConnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == nullptr) {
            controllers[i] = controller;
            lastGamepadPacketMs = millis();
            robot.gamepadArmed = false;
            gamepadNeutralSinceMs = 0;
            updateControllerConnectedFlag();
            Serial.printf("Pad podłączony, slot: %d\n", i);
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == controller) controllers[i] = nullptr;
    }

    if (robot.operatingMode == OperatingMode::AUTO) {
        pauseAuto("Pad rozłączony - AUTO PAUSE");
    } else {
        if (robot.gamepadArmed) emergencyStopMotion();
        robot.gamepadArmed = false;
        Serial.println("Pad rozłączony - MANUAL zatrzymany");
    }

    updateControllerConnectedFlag();
}

bool gamepadControlsNeutral(float leftX, float leftY, float rightX, float rightY,
                            ControllerPtr controller) {
    return leftX == 0.0f && leftY == 0.0f &&
           rightX == 0.0f && rightY == 0.0f &&
           !controller->a() && !controller->b() && !controller->miscStart();
}

bool updateGamepadArming(float leftX, float leftY, float rightX, float rightY,
                         ControllerPtr controller) {
    if (robot.operatingMode != OperatingMode::MANUAL) return false;
    if (robot.gamepadArmed) return true;

    const uint32_t now = millis();
    if (!gamepadControlsNeutral(leftX, leftY, rightX, rightY, controller)) {
        gamepadNeutralSinceMs = 0;
        return false;
    }

    if (gamepadNeutralSinceMs == 0) {
        gamepadNeutralSinceMs = now;
        return false;
    }

    if (now - gamepadNeutralSinceMs >= GAMEPAD_ARM_NEUTRAL_MS) {
        robot.gamepadArmed = true;
        previousPadStart = controller->miscStart();
        digitalWrite(PIN_LED_ERROR, LOW);
        Serial.println("Pad ARMED - MANUAL aktywny");
        return true;
    }

    return false;
}

// ============================================================
// SERWA
// ============================================================
int angleToPulseUs(float angleDeg, float minimumDeg, float maximumDeg,
                   int minimumUs, int maximumUs) {
    const float angle = clampFloat(angleDeg, minimumDeg, maximumDeg);
    const float normalized = (angle - minimumDeg) / (maximumDeg - minimumDeg);
    return lroundf(minimumUs + normalized * (maximumUs - minimumUs));
}

void updateServoTargets(float rotateCommand, float gripCommand, float deltaTimeSeconds) {
    robot.toolRotateDeg += rotateCommand * TOOL_ROTATE_SPEED_DEG_S * deltaTimeSeconds;
    robot.gripperDeg += gripCommand * GRIP_SPEED_DEG_S * deltaTimeSeconds;

    robot.toolRotateDeg = clampFloat(robot.toolRotateDeg,
                                     TOOL_ROTATE_MIN_DEG, TOOL_ROTATE_MAX_DEG);
    robot.gripperDeg = clampFloat(robot.gripperDeg,
                                  GRIP_MIN_DEG, GRIP_MAX_DEG);
}

void refreshServos() {
    if (robot.estopLatched) return;

    const uint32_t now = millis();
    if (now - lastServoUpdateMs < SERVO_PERIOD_MS) return;
    lastServoUpdateMs = now;

    servoRotateFilteredDeg +=
        (robot.toolRotateDeg - servoRotateFilteredDeg) * SERVO_FILTER_ALPHA;
    servoGripFilteredDeg +=
        (robot.gripperDeg - servoGripFilteredDeg) * SERVO_FILTER_ALPHA;

    servoRotate.writeMicroseconds(angleToPulseUs(
        servoRotateFilteredDeg,
        TOOL_ROTATE_MIN_DEG, TOOL_ROTATE_MAX_DEG,
        SERVO_ROTATE_MIN_US, SERVO_ROTATE_MAX_US));

    servoGrip.writeMicroseconds(angleToPulseUs(
        servoGripFilteredDeg,
        GRIP_MIN_DEG, GRIP_MAX_DEG,
        SERVO_GRIP_MIN_US, SERVO_GRIP_MAX_US));
}

// ============================================================
// MANUAL JOINT / TOOL
// ============================================================
void processJointMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    JointPosition target = robot.joints;
    target.arm1Deg += axisX * ARM1_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.arm2Deg += -axisY * ARM2_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.zMm += -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;
    commandManualJointPosition(target, deltaTimeSeconds);
}

void processToolMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    CartesianPosition desired = robot.tcp;
    desired.xMm += axisX * TOOL_XY_SPEED_MM_S * deltaTimeSeconds;
    desired.yMm += -axisY * TOOL_XY_SPEED_MM_S * deltaTimeSeconds;
    desired.zMm += -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;
    desired.zMm = clampFloat(desired.zMm, Z_MIN_MM, Z_MAX_MM);

    JointPosition calculatedJoints;
    constexpr bool ELBOW_UP = true;

    if (!inverseKinematics(desired.xMm, desired.yMm, ELBOW_UP, calculatedJoints)) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        return;
    }

    calculatedJoints.zMm = desired.zMm;
    digitalWrite(PIN_LED_ERROR, LOW);
    commandManualJointPosition(calculatedJoints, deltaTimeSeconds);
}

void processGamepad(float deltaTimeSeconds) {
    ControllerPtr controller = getActiveController();
    if (controller == nullptr) return;

    const uint32_t now = millis();
    if (controller->hasData()) lastGamepadPacketMs = now;

    if (now - lastGamepadPacketMs > GAMEPAD_TIMEOUT_MS) {
        if (robot.operatingMode == OperatingMode::AUTO) {
            pauseAuto("Timeout pada - AUTO PAUSE");
        } else {
            if (robot.gamepadArmed) emergencyStopMotion();
            robot.gamepadArmed = false;
            gamepadNeutralSinceMs = 0;
            digitalWrite(PIN_LED_ERROR, HIGH);
            Serial.println("Timeout pada - MANUAL zatrzymany");
        }
        return;
    }

    const float leftX  = normalizeJoystick(controller->axisX());
    const float leftY  = normalizeJoystick(controller->axisY());
    const float rightX = normalizeJoystick(controller->axisRX());
    const float rightY = normalizeJoystick(controller->axisRY());

    const bool padStart = controller->miscStart();

    if (robot.operatingMode == OperatingMode::AUTO) {
        if (padStart && !previousPadStart) toggleAutoRunPause();
        previousPadStart = padStart;
        return;
    }

    if (!updateGamepadArming(leftX, leftY, rightX, rightY, controller)) {
        previousPadStart = padStart;
        return;
    }

    if (padStart && !previousPadStart) toggleControlMode();
    previousPadStart = padStart;

    float gripCommand = 0.0f;
    if (controller->a()) gripCommand = 1.0f;
    if (controller->b()) gripCommand = -1.0f;

    updateServoTargets(rightX, gripCommand, deltaTimeSeconds);

    if (robot.controlMode == ControlMode::JOINT) {
        processJointMode(leftX, leftY, rightY, deltaTimeSeconds);
    } else {
        processToolMode(leftX, leftY, rightY, deltaTimeSeconds);
    }
}

// ============================================================
// INICJALIZACJA
// ============================================================
bool initializeSteppers() {
    stepperEngine.init(1);

    stepperArm1 = stepperEngine.stepperConnectToPin(PIN_STEP_ARM1);
    stepperArm2 = stepperEngine.stepperConnectToPin(PIN_STEP_ARM2);
    stepperZ    = stepperEngine.stepperConnectToPin(PIN_STEP_Z);

    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return false;

    stepperArm1->setDirectionPin(PIN_DIR_ARM1, true, 200);
    stepperArm2->setDirectionPin(PIN_DIR_ARM2, true, 200);
    stepperZ->setDirectionPin(PIN_DIR_Z, true, 200);

    stepperArm1->setSpeedInHz(clampStepHz(
        ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1, ARM1_MAX_STEP_HZ));
    stepperArm2->setSpeedInHz(clampStepHz(
        ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2, ARM2_MAX_STEP_HZ));
    stepperZ->setSpeedInHz(clampStepHz(
        Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z, Z_MAX_STEP_HZ));

    stepperArm1->setAcceleration(ARM1_ACCEL);
    stepperArm2->setAcceleration(ARM2_ACCEL);
    stepperZ->setAcceleration(Z_ACCEL);

    // Ręczny homing: operator ustawia robot w HOME przed włączeniem/resetem.
    stepperArm1->setCurrentPosition(arm1DegreesToSteps(HOME_ARM1_DEG));
    stepperArm2->setCurrentPosition(arm2DegreesToSteps(HOME_ARM2_DEG));
    stepperZ->setCurrentPosition(zMillimetersToSteps(HOME_Z_MM));

    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED_BT, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_BUTTON_TEACH, INPUT_PULLUP);
    pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT); // GPIO39 nie ma wewnętrznego pull-up

    digitalWrite(PIN_LED_BT, LOW);
    digitalWrite(PIN_LED_ERROR, LOW);
    digitalWrite(PIN_LED_STATUS, LOW);

    robot.controlMode = ControlMode::JOINT;
    robot.operatingMode = OperatingMode::MANUAL;
    robot.autoState = AutoState::IDLE;
    robot.estopLatched = false;
    robot.controllerConnected = false;
    robot.gamepadArmed = false;

    robot.joints = {HOME_ARM1_DEG, HOME_ARM2_DEG, HOME_Z_MM};
    robot.actualJoints = robot.joints;
    robot.tcp = forwardKinematics(robot.joints);
    robot.actualTcp = robot.tcp;
    robot.toolRotateDeg = HOME_TOOL_ROTATE_DEG;
    robot.gripperDeg = HOME_GRIP_DEG;

    servoRotate.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);
    servoRotate.attach(PIN_SERVO_ROTATE, SERVO_ROTATE_MIN_US, SERVO_ROTATE_MAX_US);
    servoGrip.attach(PIN_SERVO_GRIP, SERVO_GRIP_MIN_US, SERVO_GRIP_MAX_US);

    servoRotateFilteredDeg = robot.toolRotateDeg;
    servoGripFilteredDeg = robot.gripperDeg;

    servoRotate.writeMicroseconds(angleToPulseUs(
        servoRotateFilteredDeg,
        TOOL_ROTATE_MIN_DEG, TOOL_ROTATE_MAX_DEG,
        SERVO_ROTATE_MIN_US, SERVO_ROTATE_MAX_US));
    servoGrip.writeMicroseconds(angleToPulseUs(
        servoGripFilteredDeg,
        GRIP_MIN_DEG, GRIP_MAX_DEG,
        SERVO_GRIP_MIN_US, SERVO_GRIP_MAX_US));

    if (!initializeSteppers()) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        Serial.println("Błąd inicjalizacji FastAccelStepper");
        while (true) delay(1000);
    }

    updateActualRobotPosition();

    preferences.begin("scara-auto", false);
    loadProgram();

    BP32.setup(&onConnectedController, &onDisconnectedController);

    Serial.println();
    Serial.println("SCARA Controller - DEV AUTO/TEACH");
    Serial.println("UWAGA: HOME jest ręczny. Ustaw robot w HOME przed zasileniem/resetem.");
    Serial.println("MANUAL: START = JOINT/TOOL");
    Serial.println("TEACH: krótko = zapisz punkt, przytrzymaj 1.8 s = wyczyść program");
    Serial.println("MODE: MANUAL/AUTO");
    Serial.println("AUTO: START = start/pause/resume; program pracuje w pętli");
    Serial.printf("Wczytano %u punktów programu\n", programPointCount);
    printCurrentPosition();

    lastControlMs = millis();
}

void loop() {
    BP32.update();

    updateEmergencyStop();
    updatePanelButtons();
    updateActualRobotPosition();
    refreshServos();

    const uint32_t now = millis();
    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        float deltaTimeSeconds = static_cast<float>(now - lastControlMs) / 1000.0f;
        lastControlMs = now;
        deltaTimeSeconds = clampFloat(deltaTimeSeconds, 0.0f, MAX_CONTROL_DT_S);

        if (!robot.estopLatched) {
            processGamepad(deltaTimeSeconds);
            processAuto();
        }
    }

    delay(1);
}
