// ============================================================
// PL: RĘCZNE STEROWANIE JOINT I TOOL
// EN: MANUAL JOINT AND TOOL CONTROL
// UA: РУЧНЕ КЕРУВАННЯ JOINT І TOOL
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
        // PL: Jeżeli pad zniknie podczas ustawiania HOME Z, zapamiętaj aktualne
        //     położenie jako nowy cel względny, aby po ponownym połączeniu Z
        //     nie próbowało wracać do starego celu.
        // EN: If the gamepad disconnects while setting Z HOME, remember the current
        //     position as the new relative target so that after reconnecting Z
        //     does not try to return to the old target.
        // UA: Якщо геймпад від'єднається під час встановлення HOME осі Z, запам'ятати
        //     поточне положення як нову відносну ціль, щоб після повторного підключення Z
        //     не намагалася повернутися до старої цілі.
        if (startupZHomeRequired) syncStartupZHomeTargetToActual();

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
    // EN: Remember the D-pad state also in MANUAL so that switching to AUTO while
    //     a direction is held is not interpreted as a new press.
    // UA: Запам'ятовуй D-pad також у MANUAL, щоб перехід до AUTO з утримуваним
    //     напрямком не був розпізнаний як нове натискання.
    previousAutoDpad = dpad;

    if (controller->hasData()) lastGamepadPacketMs = now;

    if (now - lastGamepadPacketMs > GAMEPAD_TIMEOUT_MS) {
        if (robot.gamepadArmed) emergencyStopMotion();
        if (startupZHomeRequired) syncStartupZHomeTargetToActual();
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

    // PL: Po restarcie normalne sterowanie jest zablokowane do czasu ręcznego
    //     ustawienia Z. W tym etapie prawy analog Y porusza tylko osią Z,
    //     a A+B przy zatrzymanej osi ustala bieżącą pozycję jako Z=0.
    // EN: After restart, normal control stays locked until Z is manually homed.
    // UA: Після перезапуску звичайне керування заблоковане до ручного HOME осі Z.
    if (startupZHomeRequired) {
        const bool confirmZHome = controller->a() && controller->b();
        processStartupZHome(rightY, confirmZHome, deltaTimeSeconds);
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
// PL: INICJALIZACJA SILNIKÓW KROKOWYCH
// EN: STEPPER INITIALIZATION
// UA: ІНІЦІАЛІЗАЦІЯ КРОКОВИХ ДВИГУНІВ
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

    // PL: ARM1 i ARM2 nadal korzystają z ręcznego ustawienia HOME przed resetem.
    //     Dla Z ta wartość jest tylko tymczasowym zerem względnym. Po starcie
    //     operator dojeżdża osią Z padem do fizycznego HOME i zatwierdza A+B.
    // EN: ARM1/ARM2 still use manual HOME before reset. Z receives only a
    //     temporary relative zero here and is re-zeroed after A+B confirmation.
    // UA: ARM1/ARM2 як і раніше вручну встановлюються в HOME перед скиданням.
    //     Для Z тут задається лише тимчасовий відносний нуль до підтвердження A+B.
    stepperArm1->setCurrentPosition(arm1DegreesToSteps(HOME_ARM1_DEG));
    stepperArm2->setCurrentPosition(arm2DegreesToSteps(HOME_ARM2_DEG));
    stepperZ->setCurrentPosition(zMillimetersToSteps(HOME_Z_MM));

    return true;
}
