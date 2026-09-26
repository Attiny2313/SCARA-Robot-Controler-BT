// ============================================================
// PL: RĘCZNE STEROWANIE JOINT I TOOL
// EN: MANUAL JOINT AND TOOL CONTROL
// UA: РУЧНЕ КЕРУВАННЯ JOINT І TOOL
// ============================================================
bool toolTargetInitialized = false;
bool toolElbowUp = true;
bool toolLimitActive = false;

float toolWorldHeadingDeg = HOME_ARM1_DEG + HOME_ARM2_DEG + HOME_TOOL_ROTATE_DEG;

constexpr float TOOL_WORKSPACE_MARGIN_MM = 0.25f;
constexpr float TOOL_TCP_LOOKAHEAD_MM = 2.0f;
constexpr float TOOL_XY_SPEED_SCALE = 1.50f;
constexpr float TOOL_ARM_MAX_SPEED_DEG_S = 35.0f;
constexpr float TOOL_ARM_ACCEL_DEG_S2 = 180.0f;

// PL: Znak opisuje wpływ dodatniego kąta serwa obrotu na orientację narzędzia
//     względem podstawy robota. Jeżeli na gotowej mechanice kompensacja działa
//     w przeciwną stronę, wystarczy zmienić +1.0f na -1.0f.
// EN: Sign describing how positive wrist-servo motion affects world tool heading.
//     If compensation moves in the opposite direction on the real mechanism,
//     change +1.0f to -1.0f.
// UA: Знак визначає напрямок впливу серво обертання на орієнтацію інструмента.
//     Якщо компенсація працює навпаки, змінити +1.0f на -1.0f.
constexpr float TOOL_SERVO_KINEMATIC_SIGN = 1.0f;

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

// PL: Cel TCP może wyprzedzać pozycję rzeczywistą tylko o niewielką wartość.
//     Dzięki temu silniki nie kończą mikro-ruchu co 10 ms i nie zaczynają od nowa,
//     ale jednocześnie po puszczeniu analoga robot zatrzymuje się blisko miejsca
//     zwolnienia drążka.
// EN: The TCP target may lead the actual position only by a small distance.
//     This keeps the steppers moving continuously instead of restarting tiny moves
//     every control cycle, while retaining a short stop distance after stick release.
// UA: Ціль TCP може випереджати фактичну позицію лише на невелику відстань,
//     що зменшує ривки без значного вибігу після відпускання стіка.
void clampToolLookahead(const CartesianPosition& actual, CartesianPosition& target) {
    const float dx = target.xMm - actual.xMm;
    const float dy = target.yMm - actual.yMm;
    const float distance = hypotf(dx, dy);

    if (!isfinite(distance) || distance <= TOOL_TCP_LOOKAHEAD_MM || distance < 0.0001f) return;

    const float scale = TOOL_TCP_LOOKAHEAD_MM / distance;
    target.xMm = actual.xMm + dx * scale;
    target.yMm = actual.yMm + dy * scale;
}

bool calculateToolJoints(const CartesianPosition& desired, JointPosition& result) {
    return inverseKinematics(desired.xMm, desired.yMm, toolElbowUp, result);
}

// PL: W TOOL serwo obrotu jest częścią orientacji narzędzia. Zapamiętujemy
//     orientację względem podstawy: A1 + A2 + SERVO. Podczas ruchu X/Y serwo
//     kompensuje zmianę obu ramion, a prawy analog X zmienia właśnie tę orientację.
// EN: In TOOL mode the wrist servo is part of tool orientation. The controller
//     keeps A1 + A2 + SERVO approximately constant while X/Y changes; right stick X
//     changes the requested world heading instead of directly moving the servo.
// UA: У TOOL серво обертання входить до орієнтації інструмента. Контролер
//     компенсує зміну A1+A2, а правий стік X змінює заданий абсолютний кут.
void updateToolOrientationTarget(const JointPosition& targetJoints,
                                 float rotateCommand,
                                 float deltaTimeSeconds) {
    toolWorldHeadingDeg += rotateCommand * TOOL_ROTATE_SPEED_DEG_S * deltaTimeSeconds;

    const float armHeadingDeg = targetJoints.arm1Deg + targetJoints.arm2Deg;
    const float requestedServoDeg =
        (toolWorldHeadingDeg - armHeadingDeg) / TOOL_SERVO_KINEMATIC_SIGN;
    const float clampedServoDeg = clampFloat(requestedServoDeg,
                                             TOOL_ROTATE_MIN_DEG,
                                             TOOL_ROTATE_MAX_DEG);

    robot.toolRotateDeg = clampedServoDeg;

    // PL: Bez narastania błędu przy dojściu serwa do ograniczenia mechanicznego.
    // EN: Prevent heading-command wind-up while the servo is at its limit.
    // UA: Запобігати накопиченню помилки при досягненні межі серво.
    if (fabsf(clampedServoDeg - requestedServoDeg) > 0.001f) {
        toolWorldHeadingDeg = armHeadingDeg +
                              TOOL_SERVO_KINEMATIC_SIGN * clampedServoDeg;
    }
}

// PL: TOOL używa własnego sterowania osiami. Oba ramiona dostają takie samo
//     przyspieszenie kątowe i wyższą prędkość maksymalną niż zwykły JOINT.
//     Niewielkie wyprzedzenie celu TCP ogranicza efekt drgania przy ciągłej jeździe.
// EN: TOOL uses dedicated joint control. Both arms use equal angular acceleration
//     and a higher maximum speed than regular JOINT control. Small TCP lookahead
//     reduces stop-start chatter during continuous motion.
// UA: TOOL має окреме керування осями, однакове кутове прискорення обох плечей
//     та невелике випередження цілі TCP для плавнішого безперервного руху.
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

    const uint32_t arm1MaxHz = stepHz(TOOL_ARM_MAX_SPEED_DEG_S * STEPS_PER_DEG_ARM1);
    const uint32_t arm2MaxHz = stepHz(TOOL_ARM_MAX_SPEED_DEG_S * STEPS_PER_DEG_ARM2);
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

// PL: TOOL steruje TCP w X/Y; kąty ramion są obliczane przez IK. Cel TCP ma
//     niewielkie wyprzedzenie względem pozycji rzeczywistej, co ogranicza drgania
//     powodowane przez serię bardzo krótkich ruchów. Gałąź IK jest zapamiętywana
//     przy wejściu do TOOL, a serwo obrotu kompensuje zmianę A1+A2.
// EN: TOOL controls TCP X/Y through IK. A small target lookahead reduces chatter
//     from repeated micro-moves. The IK branch is latched on entry and the wrist
//     servo compensates A1+A2 to approximately preserve world tool orientation.
// UA: TOOL керує TCP X/Y через IK. Невелике випередження цілі зменшує ривки,
//     гілка IK фіксується при вході, а серво компенсує зміну A1+A2.
void processToolMode(float axisX, float axisY, float axisZ,
                     float rotateCommand, float deltaTimeSeconds) {
    updateActualRobotPosition();
    if (!jointPositionFinite(robot.actualJoints) ||
        !cartesianPositionFinite(robot.actualTcp)) {
        return;
    }

    if (!toolTargetInitialized) {
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
        toolElbowUp = robot.actualJoints.arm2Deg >= 0.0f;
        toolWorldHeadingDeg = robot.actualJoints.arm1Deg + robot.actualJoints.arm2Deg +
                              TOOL_SERVO_KINEMATIC_SIGN * robot.toolRotateDeg;
        toolTargetInitialized = true;
        setToolLimitActive(false);

        Serial.printf("TOOL SYNC: X=%.2f Y=%.2f Z=%.2f A1=%.2f A2=%.2f ROT=%.1f HEAD=%.1f branch=%s\n",
                      robot.tcp.xMm, robot.tcp.yMm, robot.tcp.zMm,
                      robot.joints.arm1Deg, robot.joints.arm2Deg,
                      robot.toolRotateDeg, toolWorldHeadingDeg,
                      toolElbowUp ? "+A2" : "-A2");
    }

    const CartesianPosition actual = robot.actualTcp;
    const float toolXySpeed = TOOL_XY_SPEED_MM_S * TOOL_XY_SPEED_SCALE;
    const float dx = axisX * toolXySpeed * deltaTimeSeconds;
    const float dy = -axisY * toolXySpeed * deltaTimeSeconds;
    const float dz = -axisZ * Z_JOG_SPEED_MM_S * deltaTimeSeconds;

    // PL: Integrujemy cel względem poprzedniego celu, ale ograniczamy jego
    //     wyprzedzenie względem rzeczywistego TCP. To daje płynniejszy ruch
    //     bez niekontrolowanego "uciekania" zadania.
    // EN: Integrate from the previous target, then limit lead over actual TCP.
    // UA: Ціль інтегрується від попередньої, але її випередження обмежується.
    CartesianPosition desired = robot.tcp;
    desired.xMm += dx;
    desired.yMm += dy;
    desired.zMm = clampFloat(robot.tcp.zMm + dz, Z_MIN_MM, Z_MAX_MM);
    clampToolLookahead(actual, desired);
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
        CartesianPosition xOnly = desired;
        xOnly.yMm = actual.yMm;
        clampToolLookahead(actual, xOnly);
        clampToolWorkspace(xOnly);
        if (calculateToolJoints(xOnly, calculatedJoints)) {
            desired = xOnly;
            xyAccepted = true;
        }
    }

    if (!xyAccepted && fabsf(dy) > 0.0001f) {
        CartesianPosition yOnly = desired;
        yOnly.xMm = actual.xMm;
        clampToolLookahead(actual, yOnly);
        clampToolWorkspace(yOnly);
        if (calculateToolJoints(yOnly, calculatedJoints)) {
            desired = yOnly;
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

    updateToolOrientationTarget(calculatedJoints, rotateCommand, deltaTimeSeconds);
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

    if (robot.controlMode == ControlMode::JOINT) {
        // PL: W JOINT prawy analog X steruje serwem obrotu bezpośrednio.
        // EN: In JOINT, right stick X directly controls the wrist servo.
        // UA: У JOINT правий стік X безпосередньо керує серво обертання.
        updateServoTargets(rightX, gripCommand, deltaTimeSeconds);
        processJointMode(leftX, leftY, rightY, deltaTimeSeconds);
    } else {
        // PL: W TOOL A/B nadal steruje chwytakiem, natomiast prawy analog X
        //     zmienia orientację narzędzia względem podstawy robota.
        // EN: In TOOL, A/B still controls grip, while right stick X changes the
        //     tool world heading handled together with SCARA kinematics.
        // UA: У TOOL A/B керує захватом, а правий стік X змінює орієнтацію
        //     інструмента відносно основи робота.
        updateServoTargets(0.0f, gripCommand, deltaTimeSeconds);
        processToolMode(leftX, leftY, rightY, rightX, deltaTimeSeconds);
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
