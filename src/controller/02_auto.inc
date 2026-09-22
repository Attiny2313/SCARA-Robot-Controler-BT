// ============================================================
// PL: PROGRAM TEACH I NVS
// EN: TEACH PROGRAM AND NVS STORAGE
// UA: ПРОГРАМА TEACH ТА ЗБЕРІГАННЯ NVS
// ============================================================
constexpr uint32_t AUTO_TOOL_ACTION_DWELL_MS = 500;

bool programPointValid(const ProgramPoint& p) {
    return jointPositionFinite(p.joints) &&
           isfinite(p.toolRotateDeg) && isfinite(p.gripperDeg) &&
           p.relayOn <= 1 &&
           p.joints.arm1Deg >= ARM1_MIN_DEG && p.joints.arm1Deg <= ARM1_MAX_DEG &&
           p.joints.arm2Deg >= ARM2_MIN_DEG && p.joints.arm2Deg <= ARM2_MAX_DEG &&
           p.joints.zMm >= Z_MIN_MM && p.joints.zMm <= Z_MAX_MM;
}

// PL: Program jest zapisywany w pamięci NVS ESP32 i pozostaje po restarcie.
// EN: The taught program is stored in ESP32 NVS and survives a restart.
// UA: Навчена програма зберігається в NVS ESP32 і залишається після перезапуску.
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
            Serial.println("NVS: niezgodny format programu - wyczyszczono");
            return;
        }

        for (uint8_t i = 0; i < programPointCount; ++i) {
            if (!programPointValid(programPoints[i])) {
                Serial.printf("NVS: P%02u zawiera NaN/Inf lub wartosc poza zakresem - wyczyszczono program\n", i + 1);
                programPointCount = 0;
                saveProgram();
                return;
            }
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
// UA: Точку можна записати лише в режимі MANUAL і після повної зупинки всіх осей.
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
    if (!jointPositionFinite(robot.actualJoints) || !cartesianPositionFinite(robot.actualTcp)) {
        Serial.println("TEACH: blad numeryczny pozycji - punkt nie zostal zapisany");
        return;
    }

    ProgramPoint& p = programPoints[programPointCount];
    p.joints = robot.actualJoints;
    p.toolRotateDeg = robot.toolRotateDeg;
    p.gripperDeg = robot.gripperDeg;
    p.relayOn = robot.relayOn ? 1 : 0;
    p.dwellMs = DEFAULT_POINT_DWELL_MS;

    if (!programPointValid(p)) {
        Serial.println("TEACH: nieprawidlowy punkt - zapis anulowany");
        return;
    }

    programPointCount++;
    saveProgram();

    Serial.printf("TEACH: zapisano P%02u  A1=%.2f A2=%.2f Z=%.2f ROT=%.1f GRIP=%.1f RELAY=%s\n",
                  programPointCount,
                  p.joints.arm1Deg, p.joints.arm2Deg, p.joints.zMm,
                  p.toolRotateDeg, p.gripperDeg,
                  p.relayOn ? "ON" : "OFF");
}

// ============================================================
// PL: TRYB AUTOMATYCZNY
// EN: AUTOMATIC MODE
// UA: АВТОМАТИЧНИЙ РЕЖИМ
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
// UA: Пауза зупиняє осі та синхронізує задане положення з лічильниками кроків,
//     тому після відновлення рух починається з фактичного положення робота.
void pauseAuto(const char* reason) {
    if (robot.operatingMode != OperatingMode::AUTO) return;
    if (robot.autoState == AutoState::IDLE ||
        robot.autoState == AutoState::FINISHED ||
        robot.autoState == AutoState::FAULT) return;

    emergencyStopMotion();
    updateActualRobotPosition();
    if (jointPositionFinite(robot.actualJoints)) {
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
    }
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

    for (uint8_t i = 0; i < programPointCount; ++i) {
        if (!programPointValid(programPoints[i])) {
            setAutoFault("AUTO: program zawiera nieprawidlowy punkt");
            return;
        }
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
    if (!programPointValid(p)) {
        setAutoFault("AUTO: punkt zawiera NaN/Inf lub jest poza zakresem");
        return;
    }

    // PL: Serwo obrotu może zmieniać pozycję podczas dojazdu. Serwo 2 (chwytak)
    //     i przekaźnik są wykonywane dopiero po osiągnięciu punktu.
    // EN: The rotation servo may move during travel. Servo 2 (gripper) and the
    //     relay are applied only after the point has been reached.
    // UA: Серво обертання може рухатися під час переміщення. Серво 2 (захват)
    //     і реле спрацьовують лише після досягнення точки.
    robot.toolRotateDeg = clampFloat(p.toolRotateDeg,
                                    TOOL_ROTATE_MIN_DEG, TOOL_ROTATE_MAX_DEG);

    commandAutoJointPosition(p.joints);

    if (!jointPositionFinite(robot.joints)) {
        setAutoFault("AUTO: blad numeryczny celu ruchu");
        return;
    }

    autoMotionStarted = true;

    Serial.printf("AUTO -> P%02u/%02u\n",
                  currentProgramPoint + 1, programPointCount);
}

// PL: Maszyna stanów AUTO: ruch -> akcja narzędzia -> postój -> kolejny punkt.
//     Jeżeli w punkcie zmienia się przekaźnik lub serwo 2, po tej zmianie
//     wymuszany jest postój co najmniej 500 ms.
// EN: AUTO state machine: move -> tool action -> dwell -> next point.
//     If the relay or servo 2 changes at a point, at least 500 ms of dwell
//     is enforced after that change.
// UA: Автомат станів AUTO: рух -> дія інструмента -> пауза -> наступна точка.
//     Якщо в точці змінюється реле або серво 2, після цієї зміни примусово
//     витримується пауза щонайменше 500 мс.
void processAuto() {
    static bool toolActionDelayRequired = false;

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
                const ProgramPoint& p = programPoints[currentProgramPoint];
                const float targetGripperDeg = clampFloat(
                    p.gripperDeg, GRIP_MIN_DEG, GRIP_MAX_DEG);
                const bool targetRelayOn = p.relayOn != 0;

                const bool gripperChanged =
                    fabsf(robot.gripperDeg - targetGripperDeg) > 0.1f;
                const bool relayChanged = robot.relayOn != targetRelayOn;

                robot.gripperDeg = targetGripperDeg;
                setRelayState(targetRelayOn);
                toolActionDelayRequired = gripperChanged || relayChanged;

                if (toolActionDelayRequired) {
                    Serial.printf("AUTO ACTION P%02u: SERVO2=%s RELAY=%s -> wait >= %lu ms\n",
                                  currentProgramPoint + 1,
                                  gripperChanged ? "MOVE" : "HOLD",
                                  relayChanged ? (targetRelayOn ? "ON" : "OFF") : "HOLD",
                                  static_cast<unsigned long>(AUTO_TOOL_ACTION_DWELL_MS));
                }

                autoMotionStarted = false;
                autoDwellStartedMs = millis();
                robot.autoState = AutoState::DWELL;
            }
            break;

        case AutoState::DWELL: {
            uint32_t dwell = programPoints[currentProgramPoint].dwellMs;
            if (toolActionDelayRequired && dwell < AUTO_TOOL_ACTION_DWELL_MS) {
                dwell = AUTO_TOOL_ACTION_DWELL_MS;
            }
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
            toolActionDelayRequired = false;
            break;
        }
    }
}

// ============================================================
// PL: E-STOP / ZATRZYMANIE AWARYJNE
// EN: E-STOP / EMERGENCY STOP
// UA: E-STOP / АВАРІЙНА ЗУПИНКА
// ============================================================
// PL: E-STOP jest zatrzaskiwany programowo. Po aktywacji wymagany jest restart.
// EN: E-STOP is latched in software. A restart is required after activation.
// UA: E-STOP фіксується програмно. Після спрацювання потрібен перезапуск контролера.
void updateEmergencyStop() {
    const bool estopActive = digitalRead(PIN_ESTOP) == HIGH;

    if (estopActive && !robot.estopLatched) {
        robot.estopLatched = true;
        robot.gamepadArmed = false;
        emergencyStopMotion();
        setRelayState(false);
        robot.autoState = AutoState::FAULT;
        digitalWrite(PIN_LED_ERROR, HIGH);
        digitalWrite(PIN_LED_STATUS, LOW);
        Serial.println("E-STOP AKTYWNY - wymagany restart sterownika; RELAY=OFF");
    }
}

// ============================================================
