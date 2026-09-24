// ============================================================
// PL: RĘCZNE STEROWANIE JOINT I TOOL
// EN: MANUAL JOINT AND TOOL CONTROL
// UA: РУЧНЕ КЕРУВАННЯ JOINT І TOOL
// ============================================================
bool toolTargetInitialized = false;
bool toolElbowUp = true;
bool toolLimitActive = false;

constexpr float TOOL_WORKSPACE_MARGIN_MM = 0.25f;
constexpr float TOOL_ARM_ACCEL_DEG_S2 = 120.0f;

void setToolLimitActive(bool active) {
    if (toolLimitActive == active) return;
    toolLimitActive = active;
    Serial.println(active ? "TOOL LIMIT: osiągnięto granicę obszaru/osi" :
                            "TOOL LIMIT: zwolniono");
}

void clampToolWorkspace(CartesianPosition& point) {
    const float radius = hypotf(point.xMm, point.yMm);
    const float minimumRadius = fabsf(ARM_LENGTH_1_MM - ARM_LENGTH_2_MM) +
                                TOOL_WORKSPACE_MARGIN_MM;
    const float maximumRadius = ARM_LENGTH_1_MM + ARM_LENGTH_2_MM -
                                TOOL_WORKSPACE_MARGIN_MM;

    if (!isfinite(radius) || radius < 0.0001f) return;

    const float clampedRadius = clampFloat(radius, minimumRadius, maximumRadius);
    if (fabsf(clampedRadius - radius) > 0.0001f) {
        const float scale = clampedRadius / radius;
        point.xMm *= scale;
        point.yMm *= scale;
    }
}

bool calculateToolJoints(const CartesianPosition& desired, JointPosition& result) {
    return inverseKinematics(desired.xMm, desired.yMm, toolElbowUp, result);
}

// PL: TOOL używa własnego sterowania osiami. Oba ramiona dostają takie samo
//     przyspieszenie kątowe, dzięki czemu podczas ruchu TCP nie rozjeżdżają się
//     tak mocno jak przy bazowych, bardzo różnych przyspieszeniach ARM1/ARM2.
// EN: TOOL uses dedicated joint control. Both arms use the same angular
//     acceleration, reducing TCP path distortion caused by unequal joint ramps.
// UA: TOOL використовує окреме керування осями. Обидва плеча мають однакове
//     кутове прискорення, що зменшує викривлення траєкторії TCP.
void commandToolJointPosition(const JointPosition& target, float deltaTimeSeconds) {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL ||
        robot.controlMode != ControlMode::TOOL) return;

    if (!jointPositionFinite(target) || deltaTimeSeconds <= 0.0f) return;

    updateActualRobotPosition();
    if (!jointPositionFinite(robot.actualJoints)) return;

    const JointPosition clamped = clampJointPosition(target);

    const float arm1Delta = fabsf(clamped.arm1Deg - robot.actualJoints.arm1Deg);
    const float arm2Delta = fabsf(clamped.arm2Deg - robot.actualJoints.arm2Deg);
    const float zDelta = fabsf(clamped.zMm - robot.actualJoints.zMm);

    const uint32_t arm1MaxHz = stepHz(ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1);
    const uint32_t arm2MaxHz = stepHz(ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2);
    const uint32_t zMaxHz = stepHz(Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z);

    if (arm1Delta > 0.0001f) {
        stepperArm1->setSpeedInHz(stepHz(fminf(
            arm1Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM1,
            static_cast<float>(arm1MaxHz))));
    }
    if (arm2Delta > 0.0001f) {
        stepperArm2->setSpeedInHz(stepHz(fminf(
            arm2Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM2,
            static_cast<float>(arm2MaxHz))));
    }
    if (zDelta > 0.0001f) {
        stepperZ->setSpeedInHz(stepHz(fminf(
            zDelta / deltaTimeSeconds * STEPS_PER_MM_Z,
            static_cast<float>(zMaxHz))));
    }

    stepperArm1->setAcceleration(stepHz(TOOL_ARM_ACCEL_DEG_S2 * STEPS_PER_DEG_ARM1));
    stepperArm2->setAcceleration(stepHz(TOOL_ARM_ACCEL_DEG_S2 * STEPS_PER_DEG_ARM2));
    stepperZ->setAcceleration(Z_ACCEL);

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

// PL: JOINT steruje osiami bezpośrednio: ARM1, ARM2 i Z.
// EN: JOINT directly controls the ARM1, ARM2 and Z axes.
// UA: JOINT безпосередньо керує осями ARM1, ARM2 і Z.
void processJointMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    // PL: Po wyjściu z TOOL następne wejście zsynchronizuje cel TCP z pozycją rzeczywistą.
    // EN: After leaving TOOL, the next TOOL entry resynchronizes the TCP target to actual position.
    // UA: Після виходу з TOOL наступний вхід у TOOL синхронізує ціль TCP з фактичним положенням.
    toolTargetInitialized = false;
    setToolLimitActive(false);

    JointPosition target = robot.joints;
    target.arm1Deg += axisX * ARM1_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.arm2Deg += -axisY * ARM2_JOG_SPEED_DEG_S * deltaTimeSeconds;
    target.zMm += -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;
    commandManualJointPosition(target, deltaTimeSeconds);
}

// PL: TOOL steruje TCP w X/Y; kąty ramion są obliczane przez IK.
//     Każda nowa komenda powstaje względem faktycznej pozycji TCP, więc cel nie
//     "ucieka" przed silnikami. Gałąź IK jest zapamiętywana przy wejściu do TOOL.
//     Przy granicy obszaru roboczego ruch jest łagodnie ograniczany zamiast
//     zapalać diodę FAULT i zatrzymywać całe sterowanie.
// EN: TOOL controls TCP in X/Y using IK. Each command is based on the actual TCP
//     position so the target cannot run away from the motors. The IK branch is
//     latched on TOOL entry. Workspace limits clamp motion instead of raising FAULT.
// UA: TOOL керує TCP у X/Y через IK. Кожна команда базується на фактичній позиції
//     TCP, гілка IK фіксується при вході, а межі робочої зони не викликають FAULT.
void processToolMode(float axisX, float axisY, float axisZ, float deltaTimeSeconds) {
    updateActualRobotPosition();
    if (!jointPositionFinite(robot.actualJoints) ||
        !cartesianPositionFinite(robot.actualTcp)) {
        return;
    }

    if (!toolTargetInitialized) {
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
        toolElbowUp = robot.actualJoints.arm2Deg >= 0.0f;
        toolTargetInitialized = true;
        setToolLimitActive(false);

        Serial.printf("TOOL SYNC: X=%.2f Y=%.2f Z=%.2f A1=%.2f A2=%.2f branch=%s\n",
                      robot.tcp.xMm, robot.tcp.yMm, robot.tcp.zMm,
                      robot.joints.arm1Deg, robot.joints.arm2Deg,
                      toolElbowUp ? "+A2" : "-A2");
    }

    const CartesianPosition actual = robot.actualTcp;
    const float dx = axisX * TOOL_XY_SPEED_MM_S * deltaTimeSeconds;
    const float dy = -axisY * TOOL_XY_SPEED_MM_S * deltaTimeSeconds;
    const float dz = -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;

    CartesianPosition desired = actual;
    desired.xMm += dx;
    desired.yMm += dy;
    desired.zMm = clampFloat(actual.zMm + dz, Z_MIN_MM, Z_MAX_MM);
    clampToolWorkspace(desired);

    JointPosition calculatedJoints;
    bool xyAccepted = calculateToolJoints(desired, calculatedJoints);

    // PL: Jeżeli ruch po przekątnej trafia w limit osi, spróbuj zachować ruch
    //     tylko w X, a następnie tylko w Y. Dzięki temu TCP nie "zawiesza się"
    //     całkowicie przy krawędzi obszaru roboczego.
    // EN: If a diagonal command hits a joint limit, try X-only and then Y-only
    //     so the TCP does not freeze completely at the workspace boundary.
    // UA: Якщо діагональний рух досягає межі осі, спробувати окремо X, потім Y.
    if (!xyAccepted && fabsf(dx) > 0.0001f) {
        CartesianPosition xOnly = actual;
        xOnly.xMm += dx;
        xOnly.zMm = desired.zMm;
        clampToolWorkspace(xOnly);
        if (calculateToolJoints(xOnly, calculatedJoints)) {
            desired.xMm = xOnly.xMm;
            desired.yMm = xOnly.yMm;
            xyAccepted = true;
        }
    }

    if (!xyAccepted && fabsf(dy) > 0.0001f) {
        CartesianPosition yOnly = actual;
        yOnly.yMm += dy;
        yOnly.zMm = desired.zMm;
        clampToolWorkspace(yOnly);
        if (calculateToolJoints(yOnly, calculatedJoints)) {
            desired.xMm = yOnly.xMm;
            desired.yMm = yOnly.yMm;
            xyAccepted = true;
        }
    }

    if (!xyAccepted) {
        // PL: Z pozostaje niezależne od ograniczenia XY. Przy niedozwolonym
        //     ruchu TCP ramiona utrzymują aktualne kąty, ale Z nadal może jechać.
        // EN: Z remains independent from an XY limit. The arms hold their actual
        //     angles while Z can still move.
        calculatedJoints = robot.actualJoints;
        desired.xMm = actual.xMm;
        desired.yMm = actual.yMm;
    }

    calculatedJoints.zMm = desired.zMm;

    const bool requestedXY = fabsf(dx) > 0.0001f || fabsf(dy) > 0.0001f;
    setToolLimitActive(requestedXY && !xyAccepted);

    commandToolJointPosition(calculatedJoints, deltaTimeSeconds);
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
    const bool padX = controller->x();
    const uint8_t dpad = controller->dpad();

    if (robot.operatingMode == OperatingMode::AUTO) {
        processAutoSpeedDpad(dpad);
        if (padStart && !previousPadStart) toggleAutoRunPause();
        previousPadStart = padStart;
        previousPadX = padX;
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
        previousPadX = padX;
        digitalWrite(PIN_LED_ERROR, HIGH);
        Serial.println("Timeout pada - MANUAL zatrzymany");
        return;
    }

    if (!updateGamepadArming(leftX, leftY, rightX, rightY, controller)) {
        previousPadStart = padStart;
        previousPadX = padX;
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
        previousPadX = padX;
        return;
    }

    if (padStart && !previousPadStart) toggleControlMode();
    previousPadStart = padStart;

    // PL: X przełącza przekaźnik tylko w MANUAL. Stan jest zapamiętywany przez TEACH.
    // EN: X toggles the relay only in MANUAL. TEACH stores the resulting relay state.
    // UA: X перемикає реле лише в MANUAL. TEACH зберігає отриманий стан реле.
    if (padX && !previousPadX) {
        setRelayState(!robot.relayOn);
        Serial.printf("MANUAL RELAY GPIO%d: %s\n", PIN_RELAY, robot.relayOn ? "ON" : "OFF");
    }
    previousPadX = padX;

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
