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

    // BT LED startuje w stanie zgaszonym; updateBluetoothLed() rozpocznie miganie.
    digitalWrite(PIN_LED_BT, BT_LED_ACTIVE_LOW ? HIGH : LOW);
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

    displayAvailable = initializeDisplay();

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
    Serial.println("AUTO SPEED: D-pad UP/DOWN = +/-10%, LEFT = 100% (50..250%)");
    Serial.println("BT LED: stale = pad polaczony, miganie = oczekiwanie");
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
    updateBluetoothLed();
    refreshServos();
    refreshDisplay();

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
