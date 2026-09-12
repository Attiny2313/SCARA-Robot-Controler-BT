/*
 * SCARA Robot Controller / Sterownik robota SCARA
 *
 * PL: Sterownik 3-osiowego robota SCARA z ręcznym sterowaniem z pada,
 *     zapisem punktów TEACH i automatycznym odtwarzaniem programu.
 * EN: Controller for a 3-axis SCARA robot with gamepad manual control,
 *     TEACH point storage and automatic program playback.
 *
 * PL: Homing jest wykonywany ręcznie. Przed uruchomieniem operator musi
 *     ustawić robota w pozycji HOME zdefiniowanej poniżej.
 * EN: Homing is manual. Before startup, the operator must place the robot
 *     in the HOME position defined below.
 */

#include <Arduino.h>
#include <Bluepad32.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ============================================================
// PINY / PIN ASSIGNMENTS
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

// PL: Wejścia krańcówek są zachowane sprzętowo na przyszłość.
//     Aktualna wersja programu używa ręcznego homingu.
// EN: Endstop inputs are kept in hardware for future use.
//     The current firmware uses manual homing.
constexpr uint8_t PIN_ENDSTOP_Z    = 34;
constexpr uint8_t PIN_ENDSTOP_ARM1 = 35;
constexpr uint8_t PIN_ENDSTOP_ARM2 = 36;

// ============================================================
// KONFIGURACJA MECHANIKI / MECHANICAL CONFIGURATION
// ============================================================
constexpr float ARM_LENGTH_1_MM = 91.61f;
constexpr float ARM_LENGTH_2_MM = 97.528f;

constexpr float MOTOR_STEPS_REV = 200.0f;
constexpr float MICROSTEPS      = 16.0f;

constexpr float GEAR_ARM1 = 3.875f;
constexpr float GEAR_ARM2 = 7.280f;
constexpr float Z_SCREW_LEAD_MM = 2.0f;
constexpr float GEAR_Z          = 1.0f;

// PL: Znak osi pozwala odwrócić kierunek logiczny bez przepinania przewodów.
// EN: Axis sign allows logical direction reversal without rewiring the motor.
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
// RĘCZNY HOME / MANUAL HOME
// ============================================================
// PL: Przed włączeniem lub resetem robot musi zostać ręcznie ustawiony
//     w pozycji HOME. Program przyjmuje te wartości jako pozycję aktualną.
// EN: Before power-up or reset the robot must be manually placed in HOME.
//     The firmware assumes these values are the actual startup position.
constexpr float HOME_ARM1_DEG = 115.0f;
constexpr float HOME_ARM2_DEG = -147.0f;
constexpr float HOME_Z_MM     = 0.0f;
constexpr float HOME_TOOL_ROTATE_DEG = 90.0f;
constexpr float HOME_GRIP_DEG        = 30.0f;

// ============================================================
// OGRANICZENIA PROGRAMOWE / SOFTWARE LIMITS
// ============================================================
// PL: Ograniczenia chronią mechanikę w trybach MANUAL, TOOL i AUTO.
// EN: These limits protect the mechanism in MANUAL, TOOL and AUTO modes.
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
// PRĘDKOŚCI I PRZYSPIESZENIA / SPEEDS AND ACCELERATIONS
// ============================================================
// PL: ARM1 i ARM2 mają tę samą prędkość kątową w sterowaniu ręcznym.
// EN: ARM1 and ARM2 use the same angular speed in manual control.
constexpr float ARM1_JOG_SPEED_DEG_S = 25.0f;
constexpr float ARM2_JOG_SPEED_DEG_S = 25.0f;

// PL: Prędkość osi Z po prawidłowym ustawieniu MS1/MS2/MS3.
// EN: Z-axis speed with MS1/MS2/MS3 configured correctly.
constexpr float Z_JOG_SPEED_MM_S     = 1.5f;
constexpr float TOOL_XY_SPEED_MM_S   = 20.0f;

// PL: Prędkości nominalne programu automatycznego.
// EN: Nominal speeds used by the automatic program.
constexpr float AUTO_ARM1_SPEED_DEG_S = 20.0f;
constexpr float AUTO_ARM2_SPEED_DEG_S = 20.0f;
constexpr float AUTO_Z_SPEED_MM_S     = 1.5f;

constexpr float TOOL_ROTATE_SPEED_DEG_S = 35.0f;
constexpr float GRIP_SPEED_DEG_S        = 45.0f;

constexpr int SERVO_ROTATE_MIN_US = 500;
constexpr int SERVO_ROTATE_MAX_US = 2500;
constexpr int SERVO_GRIP_MIN_US   = 500;
constexpr int SERVO_GRIP_MAX_US   = 2500;
constexpr float SERVO_FILTER_ALPHA = 0.20f;

// PL: Twarde limity częstotliwości STEP są dodatkowym zabezpieczeniem,
//     niezależnym od prędkości zadawanej przez JOINT, TOOL lub AUTO.
// EN: Hard STEP frequency limits provide an additional safety layer,
//     independent of speeds requested by JOINT, TOOL or AUTO control.
constexpr uint32_t ARM1_MAX_STEP_HZ = 10000;
constexpr uint32_t ARM2_MAX_STEP_HZ = 3000;

// PL: Przy 1600 STEP/mm: 2400 Hz = 1.5 mm/s.
// EN: At 1600 STEP/mm: 2400 Hz = 1.5 mm/s.
constexpr uint32_t Z_MAX_STEP_HZ    = 2400;

constexpr uint32_t ARM1_ACCEL = 12000;
constexpr uint32_t ARM2_ACCEL = 5000;

// PL: Bardzo łagodny start Z: 300 STEP/s² = 0.1875 mm/s² przy 1600 STEP/mm.
// EN: Very gentle Z start: 300 STEP/s² = 0.1875 mm/s² at 1600 STEP/mm.
constexpr uint32_t Z_ACCEL    = 300;

// ============================================================
// CZASY I AUTO / TIMING AND AUTO SETTINGS
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
// STRUKTURY I STANY / DATA TYPES AND STATES
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

// PL: Jeden punkt programu zawiera pozycję osi, pozycje serw i czas postoju.
// EN: One program point stores joint positions, servo targets and dwell time.
struct ProgramPoint {
    JointPosition joints;
    float toolRotateDeg;
    float gripperDeg;
    uint32_t dwellMs;
};

struct RobotState {
    JointPosition joints;
    JointPosition actualJoints;
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
// OBIEKTY I STAN GLOBALNY / GLOBAL OBJECTS AND STATE
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

bool previousModeButton = LOW;
bool previousTeachButton = LOW;
bool previousPadStart = false;

// ============================================================
// FUNKCJE POMOCNICZE / HELPER FUNCTIONS
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

// PL: Normalizacja osi pada do zakresu -1...+1 z martwą strefą.
// EN: Normalize a gamepad axis to -1...+1 with a dead zone around center.
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
// KINEMATYKA / KINEMATICS
// ============================================================
// PL: Kinematyka prosta: kąty obu ramion -> pozycja TCP X/Y/Z.
// EN: Forward kinematics: arm angles -> TCP position X/Y/Z.
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

// PL: Kinematyka odwrotna dla płaszczyzny XY. Parametr elbowUp wybiera
//     jedną z dwóch geometrycznie możliwych konfiguracji ramienia.
// EN: Inverse kinematics for the XY plane. elbowUp selects one of the two
//     geometrically possible arm configurations.
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
// POZYCJA I RUCH / POSITION AND MOTION
// ============================================================
// PL: Pozycja rzeczywista jest wyliczana z liczników kroków FastAccelStepper.
// EN: Actual position is calculated from FastAccelStepper step counters.
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

// PL: Natychmiastowe zatrzymanie generatorów kroków.
// EN: Immediately stop all step generators.
void emergencyStopMotion() {
    if (stepperArm1 != nullptr) stepperArm1->forceStop();
    if (stepperArm2 != nullptr) stepperArm2->forceStop();
    if (stepperZ != nullptr) stepperZ->forceStop();
}

// PL: Ruch ręczny przyjmuje małe, cyklicznie aktualizowane cele pozycji.
// EN: Manual motion receives small position targets updated every control cycle.
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

// PL: AUTO wykonuje ruch PTP. Prędkości osi są skalowane tak, aby w przybliżeniu
//     zakończyć ruch w tym samym czasie; przyspieszenia pozostają niezależne.
// EN: AUTO performs PTP motion. Axis speeds are scaled so the axes finish at
//     approximately the same time; acceleration profiles remain independent.
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
// PROGRAM TEACH I NVS / TEACH PROGRAM AND NVS STORAGE
// ============================================================
// PL: Program jest zapisywany w pamięci NVS ESP32 i pozostaje po restarcie.
// EN: The taught program is stored in ESP32 NVS and survives a restart.
void saveProgram() {
    preferences.putUChar("count", programPointCount);
    if (programPointCount > 0) {
        preferences.putBytes("points", programPoints,
                             sizeof(ProgramPoint) * programPointCount);
    } else if (preferences.isKey("points")) {
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

// PL: Punkt można zapisać tylko w MANUAL i po całkowitym zatrzymaniu osi.
// EN: A point can only be taught in MANUAL and after all axes have stopped.
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
// TRYB AUTOMATYCZNY / AUTOMATIC MODE
// ============================================================
void setAutoFault(const char* reason) {
    emergencyStopMotion();
    robot.autoState = AutoState::FAULT;
    autoMotionStarted = false;
    digitalWrite(PIN_LED_ERROR, HIGH);
    if (reason != nullptr) Serial.println(reason);
}

// PL: Pauza zatrzymuje osie i synchronizuje pozycję z licznikami kroków,
//     dzięki czemu wznowienie startuje z faktycznej pozycji robota.
// EN: Pause stops the axes and synchronizes the commanded position with the
//     step counters so resume starts from the robot's actual position.
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

// PL: Maszyna stanów AUTO: ruch -> postój -> kolejny punkt.
// EN: AUTO state machine: move -> dwell -> next point.
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
// E-STOP / EMERGENCY STOP
// ============================================================
// PL: E-STOP jest zatrzaskiwany programowo. Po aktywacji wymagany jest restart.
// EN: E-STOP is latched in software. A restart is required after activation.
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
// TRYBY I PANEL / MODES AND CONTROL PANEL
// ============================================================
void toggleControlMode() {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL) return;

    robot.controlMode =
        (robot.controlMode == ControlMode::JOINT) ? ControlMode::TOOL : ControlMode::JOINT;

    Serial.println(robot.controlMode == ControlMode::TOOL ? "MANUAL: TOOL" : "MANUAL: JOINT");
}

// PL: MODE przełącza wyłącznie przy zatrzymanych osiach.
// EN: MODE can only be changed while all axes are stopped.
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

// PL: Oba przyciski panelu są aktywne stanem HIGH i korzystają z INPUT_PULLDOWN.
//     TEACH: krótkie naciśnięcie zapisuje punkt, długie kasuje program.
// EN: Both panel buttons are active HIGH and use INPUT_PULLDOWN.
//     TEACH: short press stores a point, long press clears the program.
void updatePanelButtons() {
    const uint32_t now = millis();
    const bool modeButton = digitalRead(PIN_BUTTON_MODE);
    const bool teachButton = digitalRead(PIN_BUTTON_TEACH);

    if (previousModeButton == LOW && modeButton == HIGH &&
        now - lastModeButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastModeButtonEdgeMs = now;
        toggleOperatingMode();
    }

    if (previousTeachButton == LOW && teachButton == HIGH &&
        now - lastTeachButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastTeachButtonEdgeMs = now;
        teachPressStartedMs = now;
    }

    if (previousTeachButton == HIGH && teachButton == LOW &&
        now - lastTeachButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastTeachButtonEdgeMs = now;

        if (teachPressStartedMs != 0) {
            const uint32_t heldMs = now - teachPressStartedMs;
            if (heldMs >= TEACH_LONG_PRESS_MS) {
                clearProgram();
            } else {
                teachCurrentPoint();
            }
        }

        teachPressStartedMs = 0;
    }

    previousModeButton = modeButton;
    previousTeachButton = teachButton;
}

// ============================================================
// BLUEPAD32 I PAD / BLUEPAD32 AND GAMEPAD
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

// PL: Rozłączenie pada w AUTO zatrzymuje cykl przez PAUSE.
// EN: Gamepad disconnection during AUTO pauses the cycle.
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

// PL: Po połączeniu pad musi przez chwilę pozostawać neutralny.
//     Zapobiega to przypadkowemu ruchowi tuż po sparowaniu.
// EN: After connection the gamepad must remain neutral briefly.
//     This prevents unintended motion immediately after pairing.
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
// SERWA NARZĘDZIA / TOOL SERVOS
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

// PL: Prosty filtr pierwszego rzędu wygładza skokowe zmiany zadania serw.
// EN: A simple first-order filter smooths abrupt servo target changes.
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
// MANUAL JOINT I TOOL / MANUAL JOINT AND TOOL CONTROL
// ============================================================
// PL: JOINT steruje osiami bezpośrednio: ARM1, ARM2 i Z.
// EN: JOINT directly controls the ARM1, ARM2 and Z axes.
void processJointMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    JointPosition target = robot.joints;
    target.arm1Deg += axisX * ARM1_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.arm2Deg += -axisY * ARM2_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.zMm += -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;
    commandManualJointPosition(target, deltaTimeSeconds);
}

// PL: TOOL steruje TCP w X/Y; kąty ramion są obliczane przez IK.
// EN: TOOL controls TCP in X/Y; joint angles are calculated by IK.
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

// PL: W AUTO pad służy wyłącznie do START/PAUSE/RESUME.
//     Timeout hasData() jest używany tylko w MANUAL; brak nowych ramek nie
//     oznacza rozłączenia kontrolera podczas programu automatycznego.
// EN: In AUTO the gamepad is used only for START/PAUSE/RESUME.
//     hasData() timeout is used only in MANUAL; missing new packets does not
//     mean the controller disconnected during an automatic program.
void processGamepad(float deltaTimeSeconds) {
    ControllerPtr controller = getActiveController();

    if (controller == nullptr) {
        if (robot.operatingMode == OperatingMode::AUTO &&
            (robot.autoState == AutoState::MOVE || robot.autoState == AutoState::DWELL)) {
            pauseAuto("Brak pada - AUTO PAUSE");
        }
        return;
    }

    const uint32_t now = millis();

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

    if (controller->hasData()) lastGamepadPacketMs = now;

    if (now - lastGamepadPacketMs > GAMEPAD_TIMEOUT_MS) {
        if (robot.gamepadArmed) emergencyStopMotion();
        robot.gamepadArmed = false;
        gamepadNeutralSinceMs = 0;
        digitalWrite(PIN_LED_ERROR, HIGH);
        Serial.println("Timeout pada - MANUAL zatrzymany");
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
// INICJALIZACJA SILNIKÓW / STEPPER INITIALIZATION
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

    // PL: Ręczny homing: operator ustawia robota w HOME przed startem.
    // EN: Manual homing: operator places the robot in HOME before startup.
    stepperArm1->setCurrentPosition(arm1DegreesToSteps(HOME_ARM1_DEG));
    stepperArm2->setCurrentPosition(arm2DegreesToSteps(HOME_ARM2_DEG));
    stepperZ->setCurrentPosition(zMillimetersToSteps(HOME_Z_MM));

    return true;
}

// ============================================================
// SETUP / INITIAL SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED_BT, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_LED_STATUS, OUTPUT);

    // PL: Oba przyciski na PCB podają HIGH po wciśnięciu.
    // EN: Both PCB buttons drive the input HIGH when pressed.
    pinMode(PIN_BUTTON_TEACH, INPUT_PULLDOWN);
    pinMode(PIN_BUTTON_MODE, INPUT_PULLDOWN);

    // PL: GPIO39 nie ma wewnętrznego rezystora pull-up/pull-down.
    // EN: GPIO39 has no internal pull-up/pull-down resistor.
    pinMode(PIN_ESTOP, INPUT);

    // PL: Odczyt rzeczywistego stanu zapobiega fałszywemu zboczu po starcie.
    // EN: Reading the actual state prevents a false edge immediately after boot.
    previousTeachButton = digitalRead(PIN_BUTTON_TEACH);
    previousModeButton = digitalRead(PIN_BUTTON_MODE);

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

// ============================================================
// PĘTLA GŁÓWNA / MAIN LOOP
// ============================================================
void loop() {
    // PL: Bluepad32 musi być obsługiwany możliwie często.
    // EN: Bluepad32 should be serviced as frequently as possible.
    BP32.update();

    updateEmergencyStop();
    updatePanelButtons();
    updateActualRobotPosition();
    refreshServos();

    const uint32_t now = millis();
    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        float deltaTimeSeconds = static_cast<float>(now - lastControlMs) / 1000.0f;
        lastControlMs = now;

        // PL: Ograniczenie dt chroni przed dużym skokiem zadania po chwilowym opóźnieniu.
        // EN: Limiting dt prevents a large command jump after a temporary delay.
        deltaTimeSeconds = clampFloat(deltaTimeSeconds, 0.0f, MAX_CONTROL_DT_S);

        if (!robot.estopLatched) {
            processGamepad(deltaTimeSeconds);
            processAuto();
        }
    }

    delay(1);
}
