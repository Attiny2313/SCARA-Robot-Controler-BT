// ============================================================
// MANUAL JOINT I TOOL / MANUAL JOINT AND TOOL CONTROL
// ============================================================
// PL: JOINT steruje osiami bezpośrednio: ARM1, ARM2 i Z.
// EN: JOINT directly controls the ARM1, ARM2 and Z axes.
// UA: JOINT безпосередньо керує осями ARM1, ARM2 і Z.
void processJointMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    JointPosition target = robot.joints;
    target.arm1Deg += axisX * ARM1_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.arm2Deg += -axisY * ARM2_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.zMm += -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;
    commandManualJointPosition(target, deltaTimeSeconds);
}

// PL: TOOL steruje TCP w X/Y; kąty ramion są obliczane przez IK.
// EN: TOOL controls TCP in X/Y; joint angles are calculated by IK.
// UA: TOOL керує TCP у X/Y; кути ланок обчислюються за допомогою IK.
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

// PL: W AUTO pad służy do START/PAUSE/RESUME i regulacji prędkości całego ruchu.
//     Timeout hasData() jest używany tylko w MANUAL; brak nowych ramek nie
//     oznacza rozłączenia kontrolera podczas programu automatycznego.
// EN: In AUTO the gamepad is used for START/PAUSE/RESUME and motion speed override.
// UA: В AUTO геймпад використовується для START/PAUSE/RESUME та регулювання швидкості руху.
//     Тайм-аут hasData() використовується лише в MANUAL; відсутність нових пакетів
//     не означає відключення контролера під час виконання автоматичної програми.
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
    const uint8_t dpad = controller->dpad();

    if (robot.operatingMode == OperatingMode::AUTO) {
        processAutoSpeedDpad(dpad);
        if (padStart && !previousPadStart) toggleAutoRunPause();
        previousPadStart = padStart;
        return;
    }

    // PL: Zapamiętuj D-pad także w MANUAL, aby przejście do AUTO z przytrzymanym
    //     kierunkiem nie zostało odczytane jako nowe naciśnięcie.
    // UA: Запам'ятовуй D-pad також у MANUAL, щоб перехід до AUTO з утримуваним
    //     напрямком не був розпізнаний як нове натискання.
    previousAutoDpad = dpad;

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

    stepperArm1->setSpeedInHz(stepHz(
        ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1));
    stepperArm2->setSpeedInHz(stepHz(
        ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2));
    stepperZ->setSpeedInHz(stepHz(
        Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z));

    stepperArm1->setAcceleration(ARM1_ACCEL);
    stepperArm2->setAcceleration(ARM2_ACCEL);
    stepperZ->setAcceleration(Z_ACCEL);

    // PL: Ręczny homing: operator ustawia robota w HOME przed startem.
    // EN: Manual homing: operator places the robot in HOME before startup.
    // UA: Ручне встановлення HOME: оператор встановлює робота в HOME перед запуском.
    stepperArm1->setCurrentPosition(arm1DegreesToSteps(HOME_ARM1_DEG));
    stepperArm2->setCurrentPosition(arm2DegreesToSteps(HOME_ARM2_DEG));
    stepperZ->setCurrentPosition(zMillimetersToSteps(HOME_Z_MM));

    return true;
}
