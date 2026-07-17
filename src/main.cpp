#include <Arduino.h>
#include <Bluepad32.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>

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

// Przewidziane na później:
constexpr uint8_t PIN_ENDSTOP_Z    = 34;
constexpr uint8_t PIN_ENDSTOP_ARM1 = 35;
constexpr uint8_t PIN_ENDSTOP_ARM2 = 36;

// ============================================================
// KONFIGURACJA MECHANIKI
// WARTOŚCI KONIECZNIE DOSTOSUJ DO SWOJEGO ROBOTA
// ============================================================

// Długości ramion od osi obrotu do osi obrotu [mm].
constexpr float ARM_LENGTH_1_MM = 91.61f;
constexpr float ARM_LENGTH_2_MM = 97.528f;

// Silnik 200 kroków/obrót, mikrokrok 1/16:
constexpr float MOTOR_STEPS_REV = 200.0f;
constexpr float MICROSTEPS      = 16.0f;

// Przełożenia: obrót silnika / obrót osi.
// Przykład: koło silnika 16T i koło osi 80T => 80/16 = 5.
constexpr float GEAR_ARM1 = 3.875f;
constexpr float GEAR_ARM2 = 7.280f;

// Śruba osi Z: 2 mm na obrót.
constexpr float Z_SCREW_LEAD_MM = 2.0f;
constexpr float GEAR_Z          = 1.0f;

// Odwrócenie kierunku osi.
// Zmień na -1, jeśli dana oś jedzie przeciwnie.
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
// POZYCJA DOMOWA PO WŁĄCZENIU
// ============================================================

// Program zakłada, że po załączeniu zasilania robot naprawdę znajduje się
// dokładnie w tej pozycji.
constexpr float HOME_ARM1_DEG = 115.0f;  //sprawdzić znak
constexpr float HOME_ARM2_DEG = -147.0f; //sprawdzić znak
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

// Tryb JOINT:
constexpr float ARM_JOG_SPEED_DEG_S = 25.0f;
constexpr float Z_JOG_SPEED_MM_S    = 15.0f;

// Tryb TOOL:
constexpr float TOOL_XY_SPEED_MM_S  = 60.0f;

// Serwa:
constexpr float TOOL_ROTATE_SPEED_DEG_S = 70.0f;
constexpr float GRIP_SPEED_DEG_S        = 90.0f;

// Maksymalne częstotliwości STEP:
constexpr uint32_t ARM1_MAX_STEP_HZ = 10000;
constexpr uint32_t ARM2_MAX_STEP_HZ = 10000;
constexpr uint32_t Z_MAX_STEP_HZ    = 12000;

// Przyspieszenie w krokach/s²:
constexpr uint32_t ARM1_ACCEL = 12000;
constexpr uint32_t ARM2_ACCEL = 12000;
constexpr uint32_t Z_ACCEL    = 15000;

// ============================================================
// CZASY
// ============================================================

constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t GAMEPAD_TIMEOUT_MS = 500;
constexpr int16_t JOYSTICK_DEADZONE = 60;

// ============================================================
// STRUKTURY
// ============================================================

enum class ControlMode : uint8_t {
    JOINT,
    TOOL
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

struct RobotState {
    JointPosition joints;
    CartesianPosition tcp;

    float toolRotateDeg;
    float gripperDeg;

    ControlMode mode;

    bool estopLatched;
    bool controllerConnected;
};

// ============================================================
// OBIEKTY
// ============================================================

FastAccelStepperEngine stepperEngine;

FastAccelStepper* stepperArm1 = nullptr;
FastAccelStepper* stepperArm2 = nullptr;
FastAccelStepper* stepperZ    = nullptr;

Servo servoRotate;
Servo servoGrip;

ControllerPtr controllers[BP32_MAX_GAMEPADS];

RobotState robot;

// ============================================================
// ZMIENNE CZASOWE
// ============================================================

uint32_t lastControlMs = 0;
uint32_t lastGamepadPacketMs = 0;

bool previousModeButton = HIGH;
bool previousTeachButton = HIGH;
bool previousPadStart = false;

// ============================================================
// FUNKCJE POMOCNICZE
// ============================================================

float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

float degToRad(float degrees) {
    return degrees * PI / 180.0f;
}

float radToDeg(float radians) {
    return radians * 180.0f / PI;
}

float normalizeJoystick(int32_t value) {
    if (abs(value) < JOYSTICK_DEADZONE) {
        return 0.0f;
    }

    constexpr float maximumAxisValue = 512.0f;

    float normalized = static_cast<float>(value) / maximumAxisValue;
    return clampFloat(normalized, -1.0f, 1.0f);
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
    return static_cast<float>(steps) /
           (STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

float arm2StepsToDegrees(int32_t steps) {
    return static_cast<float>(steps) /
           (STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

float zStepsToMillimeters(int32_t steps) {
    return static_cast<float>(steps) /
           (STEPS_PER_MM_Z * SIGN_Z);
}

// ============================================================
// KINEMATYKA PROSTA
// ============================================================

CartesianPosition forwardKinematics(const JointPosition& joints) {
    const float theta1 = degToRad(joints.arm1Deg);
    const float theta2 = degToRad(joints.arm2Deg);

    CartesianPosition result;

    result.xMm =
        ARM_LENGTH_1_MM * cosf(theta1) +
        ARM_LENGTH_2_MM * cosf(theta1 + theta2);

    result.yMm =
        ARM_LENGTH_1_MM * sinf(theta1) +
        ARM_LENGTH_2_MM * sinf(theta1 + theta2);

    result.zMm = joints.zMm;

    return result;
}

// ============================================================
// KINEMATYKA ODWROTNA
// ============================================================

bool inverseKinematics(
    float xMm,
    float yMm,
    bool elbowUp,
    JointPosition& result
) {
    const float radiusSquared = xMm * xMm + yMm * yMm;

    float cosTheta2 =
        (radiusSquared -
         ARM_LENGTH_1_MM * ARM_LENGTH_1_MM -
         ARM_LENGTH_2_MM * ARM_LENGTH_2_MM) /
        (2.0f * ARM_LENGTH_1_MM * ARM_LENGTH_2_MM);

    if (cosTheta2 < -1.0f || cosTheta2 > 1.0f) {
        return false;
    }

    cosTheta2 = clampFloat(cosTheta2, -1.0f, 1.0f);

    float sinTheta2 = sqrtf(1.0f - cosTheta2 * cosTheta2);

    if (!elbowUp) {
        sinTheta2 = -sinTheta2;
    }

    const float theta2 = atan2f(sinTheta2, cosTheta2);

    const float theta1 =
        atan2f(yMm, xMm) -
        atan2f(
            ARM_LENGTH_2_MM * sinTheta2,
            ARM_LENGTH_1_MM +
                ARM_LENGTH_2_MM * cosTheta2
        );

    result.arm1Deg = radToDeg(theta1);
    result.arm2Deg = radToDeg(theta2);
    result.zMm = robot.joints.zMm;

    if (result.arm1Deg < ARM1_MIN_DEG ||
        result.arm1Deg > ARM1_MAX_DEG ||
        result.arm2Deg < ARM2_MIN_DEG ||
        result.arm2Deg > ARM2_MAX_DEG) {
        return false;
    }

    return true;
}

// ============================================================
// STEROWANIE SILNIKAMI
// ============================================================

void commandJointPosition(const JointPosition& target) {
    if (robot.estopLatched) {
        return;
    }

    const float arm1 =
        clampFloat(target.arm1Deg, ARM1_MIN_DEG, ARM1_MAX_DEG);

    const float arm2 =
        clampFloat(target.arm2Deg, ARM2_MIN_DEG, ARM2_MAX_DEG);

    const float z =
        clampFloat(target.zMm, Z_MIN_MM, Z_MAX_MM);

    stepperArm1->moveTo(arm1DegreesToSteps(arm1));
    stepperArm2->moveTo(arm2DegreesToSteps(arm2));
    stepperZ->moveTo(zMillimetersToSteps(z));

    robot.joints.arm1Deg = arm1;
    robot.joints.arm2Deg = arm2;
    robot.joints.zMm = z;

    robot.tcp = forwardKinematics(robot.joints);
}

void emergencyStopMotion() {
    if (stepperArm1 != nullptr) {
        stepperArm1->forceStop();
    }

    if (stepperArm2 != nullptr) {
        stepperArm2->forceStop();
    }

    if (stepperZ != nullptr) {
        stepperZ->forceStop();
    }
}

// ============================================================
// E-STOP
// ============================================================

void updateEmergencyStop() {
    // Założenie:
    // styk NC zwiera wejście do GND podczas prawidłowej pracy,
    // a po naciśnięciu E-STOP wejście przechodzi w HIGH
    // przez zewnętrzny rezystor pull-up.
    const bool estopActive = digitalRead(PIN_ESTOP) == HIGH;

    if (estopActive && !robot.estopLatched) {
        robot.estopLatched = true;

        emergencyStopMotion();

        digitalWrite(PIN_LED_ERROR, HIGH);
        digitalWrite(PIN_LED_STATUS, LOW);

        Serial.println("E-STOP AKTYWNY");
    }
}

// ============================================================
// TRYB STEROWANIA
// ============================================================

void toggleControlMode() {
    if (robot.estopLatched) {
        return;
    }

    if (robot.mode == ControlMode::JOINT) {
        robot.mode = ControlMode::TOOL;
        Serial.println("Tryb: TOOL");
    } else {
        robot.mode = ControlMode::JOINT;
        Serial.println("Tryb: JOINT");
    }

    digitalWrite(
        PIN_LED_STATUS,
        robot.mode == ControlMode::TOOL ? HIGH : LOW
    );
}

void printCurrentPosition() {
    Serial.println();
    Serial.println("----- POZYCJA ROBOTA -----");

    Serial.printf(
        "Ramię 1: %.2f deg\n",
        robot.joints.arm1Deg
    );

    Serial.printf(
        "Ramię 2: %.2f deg\n",
        robot.joints.arm2Deg
    );

    Serial.printf(
        "Z: %.2f mm\n",
        robot.joints.zMm
    );

    Serial.printf(
        "TCP X: %.2f mm\n",
        robot.tcp.xMm
    );

    Serial.printf(
        "TCP Y: %.2f mm\n",
        robot.tcp.yMm
    );

    Serial.printf(
        "Obrót chwytaka: %.1f deg\n",
        robot.toolRotateDeg
    );

    Serial.printf(
        "Chwytak: %.1f deg\n",
        robot.gripperDeg
    );

    Serial.println("--------------------------");
}

// ============================================================
// PRZYCISKI NA PCB
// ============================================================

void updatePanelButtons() {
    const bool modeButton = digitalRead(PIN_BUTTON_MODE);
    const bool teachButton = digitalRead(PIN_BUTTON_TEACH);

    if (previousModeButton == HIGH && modeButton == LOW) {
        toggleControlMode();
    }

    if (previousTeachButton == HIGH && teachButton == LOW) {
        printCurrentPosition();
    }

    previousModeButton = modeButton;
    previousTeachButton = teachButton;
}

// ============================================================
// BLUEPAD32
// ============================================================

void onConnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == nullptr) {
            controllers[i] = controller;
            robot.controllerConnected = true;
            lastGamepadPacketMs = millis();

            digitalWrite(PIN_LED_BT, HIGH);

            Serial.printf(
                "Pad podłączony, slot: %d\n",
                i
            );

            return;
        }
    }
}

void onDisconnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == controller) {
            controllers[i] = nullptr;
        }
    }

    robot.controllerConnected = false;
    digitalWrite(PIN_LED_BT, LOW);

    emergencyStopMotion();

    Serial.println("Pad rozłączony");
}

ControllerPtr getActiveController() {
    for (ControllerPtr controller : controllers) {
        if (controller != nullptr &&
            controller->isConnected() &&
            controller->hasData() &&
            controller->isGamepad()) {
            return controller;
        }
    }

    return nullptr;
}

// ============================================================
// SERWA
// ============================================================

void updateServos(
    float rotateCommand,
    float gripCommand,
    float deltaTimeSeconds
) {
    robot.toolRotateDeg +=
        rotateCommand *
        TOOL_ROTATE_SPEED_DEG_S *
        deltaTimeSeconds;

    robot.gripperDeg +=
        gripCommand *
        GRIP_SPEED_DEG_S *
        deltaTimeSeconds;

    robot.toolRotateDeg = clampFloat(
        robot.toolRotateDeg,
        TOOL_ROTATE_MIN_DEG,
        TOOL_ROTATE_MAX_DEG
    );

    robot.gripperDeg = clampFloat(
        robot.gripperDeg,
        GRIP_MIN_DEG,
        GRIP_MAX_DEG
    );

    servoRotate.write(
        static_cast<int>(robot.toolRotateDeg)
    );

    servoGrip.write(
        static_cast<int>(robot.gripperDeg)
    );
}

// ============================================================
// JOINT MODE
// ============================================================

void processJointMode(
    float axisX,
    float axisY,
    float axisZ,
    float deltaTimeSeconds
) {
    JointPosition target = robot.joints;

    target.arm1Deg +=
        axisX *
        ARM_JOG_SPEED_DEG_S *
        deltaTimeSeconds;

    // Odwrócenie Y, aby ruch drążka do góry zwiększał wartość.
    target.arm2Deg +=
        -axisY *
        ARM_JOG_SPEED_DEG_S *
        deltaTimeSeconds;

    target.zMm +=
        -axisZ *
        Z_JOG_SPEED_MM_S *
        deltaTimeSeconds;

    commandJointPosition(target);
}

// ============================================================
// TOOL MODE
// ============================================================

void processToolMode(
    float axisX,
    float axisY,
    float axisZ,
    float deltaTimeSeconds
) {
    CartesianPosition desired = robot.tcp;

    desired.xMm +=
        axisX *
        TOOL_XY_SPEED_MM_S *
        deltaTimeSeconds;

    desired.yMm +=
        -axisY *
        TOOL_XY_SPEED_MM_S *
        deltaTimeSeconds;

    desired.zMm +=
        -axisZ *
        Z_JOG_SPEED_MM_S *
        deltaTimeSeconds;

    desired.zMm =
        clampFloat(desired.zMm, Z_MIN_MM, Z_MAX_MM);

    JointPosition calculatedJoints;

    // true = wybrana konfiguracja łokcia.
    // Zmień na false, jeśli robot ma pracować w drugiej konfiguracji.
    constexpr bool ELBOW_UP = true;

    if (!inverseKinematics(
            desired.xMm,
            desired.yMm,
            ELBOW_UP,
            calculatedJoints)) {
        digitalWrite(PIN_LED_ERROR, HIGH);
        return;
    }

    calculatedJoints.zMm = desired.zMm;

    digitalWrite(PIN_LED_ERROR, LOW);

    commandJointPosition(calculatedJoints);
}

// ============================================================
// OBSŁUGA PADA
// ============================================================

void processGamepad(float deltaTimeSeconds) {
    ControllerPtr controller = getActiveController();

    if (controller == nullptr) {
        return;
    }

    lastGamepadPacketMs = millis();

    const float leftX =
        normalizeJoystick(controller->axisX());

    const float leftY =
        normalizeJoystick(controller->axisY());

    const float rightX =
        normalizeJoystick(controller->axisRX());

    const float rightY =
        normalizeJoystick(controller->axisRY());

    const bool padStart =
        controller->miscStart();

    if (padStart && !previousPadStart) {
        toggleControlMode();
    }

    previousPadStart = padStart;

    float gripCommand = 0.0f;

    if (controller->a()) {
        gripCommand = 1.0f;
    }

    if (controller->b()) {
        gripCommand = -1.0f;
    }

    updateServos(
        rightX,
        gripCommand,
        deltaTimeSeconds
    );

    if (robot.mode == ControlMode::JOINT) {
        processJointMode(
            leftX,
            leftY,
            rightY,
            deltaTimeSeconds
        );
    } else {
        processToolMode(
            leftX,
            leftY,
            rightY,
            deltaTimeSeconds
        );
    }
}

// ============================================================
// INICJALIZACJA SILNIKÓW
// ============================================================

bool initializeSteppers() {
    stepperEngine.init(1);

    stepperArm1 =
        stepperEngine.stepperConnectToPin(PIN_STEP_ARM1);

    stepperArm2 =
        stepperEngine.stepperConnectToPin(PIN_STEP_ARM2);

    stepperZ =
        stepperEngine.stepperConnectToPin(PIN_STEP_Z);

    if (stepperArm1 == nullptr ||
        stepperArm2 == nullptr ||
        stepperZ == nullptr) {
        return false;
    }

    stepperArm1->setDirectionPin(
        PIN_DIR_ARM1,
        true,
        200
    );

    stepperArm2->setDirectionPin(
        PIN_DIR_ARM2,
        true,
        200
    );

    stepperZ->setDirectionPin(
        PIN_DIR_Z,
        true,
        200
    );

    stepperArm1->setSpeedInHz(ARM1_MAX_STEP_HZ);
    stepperArm2->setSpeedInHz(ARM2_MAX_STEP_HZ);
    stepperZ->setSpeedInHz(Z_MAX_STEP_HZ);

    stepperArm1->setAcceleration(ARM1_ACCEL);
    stepperArm2->setAcceleration(ARM2_ACCEL);
    stepperZ->setAcceleration(Z_ACCEL);

    stepperArm1->setCurrentPosition(
        arm1DegreesToSteps(HOME_ARM1_DEG)
    );

    stepperArm2->setCurrentPosition(
        arm2DegreesToSteps(HOME_ARM2_DEG)
    );

    stepperZ->setCurrentPosition(
        zMillimetersToSteps(HOME_Z_MM)
    );

    return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LED_BT, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_LED_STATUS, OUTPUT);

    pinMode(PIN_BUTTON_TEACH, INPUT_PULLUP);
    pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);

    // GPIO39 nie ma wewnętrznego pull-up.
    // Rezystor musi być na PCB.
    pinMode(PIN_ESTOP, INPUT);

    digitalWrite(PIN_LED_BT, LOW);
    digitalWrite(PIN_LED_ERROR, LOW);
    digitalWrite(PIN_LED_STATUS, LOW);

    robot.mode = ControlMode::JOINT;
    robot.estopLatched = false;
    robot.controllerConnected = false;

    robot.joints.arm1Deg = HOME_ARM1_DEG;
    robot.joints.arm2Deg = HOME_ARM2_DEG;
    robot.joints.zMm = HOME_Z_MM;

    robot.tcp = forwardKinematics(robot.joints);

    robot.toolRotateDeg = HOME_TOOL_ROTATE_DEG;
    robot.gripperDeg = HOME_GRIP_DEG;

    servoRotate.setPeriodHertz(50);
    servoGrip.setPeriodHertz(50);

    servoRotate.attach(
        PIN_SERVO_ROTATE,
        500,
        2500
    );

    servoGrip.attach(
        PIN_SERVO_GRIP,
        500,
        2500
    );

    servoRotate.write(
        static_cast<int>(robot.toolRotateDeg)
    );

    servoGrip.write(
        static_cast<int>(robot.gripperDeg)
    );

    if (!initializeSteppers()) {
        digitalWrite(PIN_LED_ERROR, HIGH);

        Serial.println(
            "Błąd inicjalizacji FastAccelStepper"
        );

        while (true) {
            delay(1000);
        }
    }

    BP32.setup(
        &onConnectedController,
        &onDisconnectedController
    );

    // Nie wywołuj BP32.forgetBluetoothKeys() przy każdym starcie,
    // bo pad nie będzie się automatycznie ponownie łączył.

    Serial.println();
    Serial.println("Sterownik SCARA uruchomiony");
    Serial.println("Pozycja po włączeniu została uznana za HOME");

    printCurrentPosition();
}


// ============================================================
// LOOP
// ============================================================

void loop() {
    BP32.update();

    updateEmergencyStop();
    updatePanelButtons();

    const uint32_t now = millis();

    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        const float deltaTimeSeconds =
            static_cast<float>(now - lastControlMs) / 1000.0f;

        lastControlMs = now;

        if (!robot.estopLatched) {
            processGamepad(deltaTimeSeconds);
        }
    }

    if (robot.controllerConnected &&
        millis() - lastGamepadPacketMs > GAMEPAD_TIMEOUT_MS) {
        emergencyStopMotion();

        digitalWrite(PIN_LED_ERROR, HIGH);
    }

    delay(1);
}