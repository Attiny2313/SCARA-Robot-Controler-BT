// PL: TRYBY I PANEL
// EN: MODES AND CONTROL PANEL
// UA: РЕЖИМИ ТА ПАНЕЛЬ КЕРУВАННЯ
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
// UA: MODE можна перемикати лише тоді, коли всі осі зупинені.
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
        previousPadX = false;
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
        previousPadX = false;
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
    Serial.printf("RELAY GPIO%d: %s\n", PIN_RELAY, robot.relayOn ? "ON" : "OFF");
    Serial.printf("WORK: %s  MANUAL: %s\n",
                  robot.operatingMode == OperatingMode::AUTO ? "AUTO" : "MANUAL",
                  robot.controlMode == ControlMode::TOOL ? "TOOL" : "JOINT");
    Serial.printf("TEACH points: %u/%u\n", programPointCount, MAX_PROGRAM_POINTS);
    Serial.printf("AUTO speed: %u%%\n", autoArmSpeedPercent);
    Serial.printf("Pad: %s\n", robot.gamepadArmed ? "ARMED" : "SAFE");
    Serial.println("-----------------------");
}

bool teachLongPressHandled = false;

// PL: Oba przyciski panelu są aktywne stanem HIGH i korzystają z INPUT_PULLDOWN.
//     TEACH: krótkie naciśnięcie zapisuje punkt, długie kasuje program dokładnie
//     po osiągnięciu progu TEACH_LONG_PRESS_MS, bez czekania na puszczenie przycisku.
// EN: Both panel buttons are active HIGH and use INPUT_PULLDOWN.
//     TEACH: short press stores a point; long press clears the program as soon as
//     TEACH_LONG_PRESS_MS is reached, without waiting for button release.
// UA: Обидві кнопки панелі активні рівнем HIGH і використовують INPUT_PULLDOWN.
//     TEACH: коротке натискання зберігає точку, довге очищає програму одразу після
//     досягнення TEACH_LONG_PRESS_MS, без очікування відпускання кнопки.
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
        teachLongPressHandled = false;
    }

    // PL: Długie przytrzymanie kasuje program w chwili osiągnięcia progu czasu.
    //     clearProgram() ustawia jednocześnie powiadomienie OLED, więc komunikat
    //     "PROGRAM SKASOWANY" pojawia się wtedy, gdy kasowanie faktycznie nastąpiło.
    // EN: Long press clears the program immediately when the time threshold is reached.
    //     clearProgram() also starts the OLED notice at the exact same moment.
    // UA: Довге утримання очищає програму одразу після досягнення порогу часу.
    //     clearProgram() одночасно запускає повідомлення OLED.
    if (teachButton == HIGH && teachPressStartedMs != 0 && !teachLongPressHandled &&
        now - teachPressStartedMs >= TEACH_LONG_PRESS_MS) {
        teachLongPressHandled = true;
        clearProgram();
    }

    if (previousTeachButton == HIGH && teachButton == LOW &&
        now - lastTeachButtonEdgeMs >= BUTTON_DEBOUNCE_MS) {
        lastTeachButtonEdgeMs = now;

        if (teachPressStartedMs != 0 && !teachLongPressHandled) {
            teachCurrentPoint();
        }

        teachPressStartedMs = 0;
        teachLongPressHandled = false;
    }

    previousModeButton = modeButton;
    previousTeachButton = teachButton;
}

// ============================================================
// PL: BLUEPAD32 I PAD
// EN: BLUEPAD32 AND GAMEPAD
// UA: BLUEPAD32 ТА ГЕЙМПАД
// ============================================================
ControllerPtr getActiveController() {
    for (ControllerPtr controller : controllers) {
        if (controller != nullptr && controller->isConnected() && controller->isGamepad()) {
            return controller;
        }
    }
    return nullptr;
}

void writeBtLed(bool on) {
    const bool pinHigh = BT_LED_ACTIVE_LOW ? !on : on;
    digitalWrite(PIN_LED_BT, pinHigh ? HIGH : LOW);
}

void updateControllerConnectedFlag() {
    robot.controllerConnected = getActiveController() != nullptr;

    if (robot.controllerConnected) {
        btLedBlinkState = true;
        writeBtLed(true);
    } else {
        btLedBlinkState = false;
        lastBtLedBlinkMs = millis();
        writeBtLed(false);
    }
}

// PL: Połączony pad = światło ciągłe. Brak pada = miganie co 500 ms.
// EN: Connected controller = steady LED. No controller = 500 ms blink.
// UA: Підключений геймпад = постійне світіння. Немає геймпада = миготіння кожні 500 мс.
void updateBluetoothLed() {
    const bool connectedNow = getActiveController() != nullptr;
    robot.controllerConnected = connectedNow;

    if (connectedNow) {
        btLedBlinkState = true;
        writeBtLed(true);
        return;
    }

    const uint32_t now = millis();
    if (now - lastBtLedBlinkMs >= BT_LED_BLINK_MS) {
        lastBtLedBlinkMs = now;
        btLedBlinkState = !btLedBlinkState;
        writeBtLed(btLedBlinkState);
    }
}

void onConnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == nullptr) {
            controllers[i] = controller;
            lastGamepadPacketMs = millis();
            robot.gamepadArmed = false;
            gamepadNeutralSinceMs = 0;
            previousPadStart = false;
            previousPadX = false;
            updateControllerConnectedFlag();
            Serial.printf("Pad podłączony, slot: %d\n", i);
            return;
        }
    }
}

// PL: Rozłączenie pada w AUTO zatrzymuje cykl przez PAUSE.
// EN: Gamepad disconnection during AUTO pauses the cycle.
// UA: Відключення геймпада в AUTO переводить цикл у PAUSE.
void onDisconnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (controllers[i] == controller) controllers[i] = nullptr;
    }

    previousPadStart = false;
    previousPadX = false;

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
           !controller->a() && !controller->b() && !controller->x() &&
           !controller->miscStart();
}

// PL: Po połączeniu pad musi przez chwilę pozostawać neutralny.
//     Zapobiega to przypadkowemu ruchowi lub przełączeniu przekaźnika tuż po sparowaniu.
// EN: After connection the gamepad must remain neutral briefly.
//     This prevents unintended motion or relay switching immediately after pairing.
// UA: Після підключення геймпад повинен короткий час залишатися нейтральним.
//     Це запобігає випадковому руху або перемиканню реле одразу після спарювання.
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
        previousPadX = controller->x();
        digitalWrite(PIN_LED_ERROR, LOW);
        Serial.println("Pad ARMED - MANUAL aktywny");
        return true;
    }

    return false;
}
