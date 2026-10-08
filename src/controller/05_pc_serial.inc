// ============================================================
// PL: STEROWANIE Z APLIKACJI PC PRZEZ USB SERIAL
// EN: PC CONTROL OVER USB SERIAL
// UA: КЕРУВАННЯ З ПК ЧЕРЕЗ USB SERIAL
// ============================================================
//
// PL: Protokół wejściowy:
//     PCHELLO
//     PCPAD,LX,LY,RX,RY,BUTTONS,DPAD
//     PCTEACH
//     PCCLEAR
//     PCBYE
//
//     Osie są liczbami całkowitymi -1000..1000.
//     BUTTONS: bit0=A, bit1=B, bit2=X, bit3=START, bit4=MODE, bit5=TEACH.
//     DPAD: bit0=UP, bit1=DOWN, bit2=LEFT, bit3=RIGHT.
//
// EN: Input protocol:
//     PCHELLO
//     PCPAD,LX,LY,RX,RY,BUTTONS,DPAD
//     PCTEACH
//     PCCLEAR
//     PCBYE
//
//     Axes are integers in the -1000..1000 range.
//     BUTTONS: bit0=A, bit1=B, bit2=X, bit3=START, bit4=MODE, bit5=TEACH.
//     DPAD: bit0=UP, bit1=DOWN, bit2=LEFT, bit3=RIGHT.
//
// UA: Вхідний протокол:
//     PCHELLO
//     PCPAD,LX,LY,RX,RY,BUTTONS,DPAD
//     PCTEACH
//     PCCLEAR
//     PCBYE
//
//     Осі задаються цілими числами -1000..1000.
//     BUTTONS: bit0=A, bit1=B, bit2=X, bit3=START, bit4=MODE, bit5=TEACH.
//     DPAD: bit0=UP, bit1=DOWN, bit2=LEFT, bit3=RIGHT.

constexpr uint16_t SERIAL_BTN_A     = 1u << 0;
constexpr uint16_t SERIAL_BTN_B     = 1u << 1;
constexpr uint16_t SERIAL_BTN_X     = 1u << 2;
constexpr uint16_t SERIAL_BTN_START = 1u << 3;
constexpr uint16_t SERIAL_BTN_MODE  = 1u << 4;
constexpr uint16_t SERIAL_BTN_TEACH = 1u << 5;

constexpr uint8_t SERIAL_DPAD_UP    = 1u << 0;
constexpr uint8_t SERIAL_DPAD_DOWN  = 1u << 1;
constexpr uint8_t SERIAL_DPAD_LEFT  = 1u << 2;
constexpr uint8_t SERIAL_DPAD_RIGHT = 1u << 3;

constexpr uint32_t SERIAL_CONTROL_TIMEOUT_MS = 600;
constexpr uint32_t SERIAL_CONTROL_NEUTRAL_MS = GAMEPAD_ARM_NEUTRAL_MS;
constexpr uint32_t SERIAL_STATUS_PERIOD_MS = 200;
constexpr size_t SERIAL_RX_LINE_MAX = 128;

struct SerialPadState {
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    uint16_t buttons;
    uint8_t dpad;
};

SerialPadState serialPad = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0};

bool serialControlSessionActive = false;
uint32_t serialControlLastPacketMs = 0;
uint32_t serialControlNeutralSinceMs = 0;
uint32_t serialControlLastStatusMs = 0;
uint16_t serialPreviousButtons = 0;
uint8_t serialPreviousDpad = 0;
uint32_t serialTeachPressStartedMs = 0;
bool serialTeachLongPressHandled = false;

char serialRxLine[SERIAL_RX_LINE_MAX];
size_t serialRxLength = 0;

float serialAxisFromInt(int value) {
    value = constrain(value, -1000, 1000);
    const float normalized = static_cast<float>(value) / 1000.0f;
    return fabsf(normalized) < 0.03f ? 0.0f : normalized;
}

uint8_t serialDpadToBluepad(uint8_t serialDpad) {
    uint8_t result = 0;
    if (serialDpad & SERIAL_DPAD_UP) result |= DPAD_UP;
    if (serialDpad & SERIAL_DPAD_DOWN) result |= DPAD_DOWN;
    if (serialDpad & SERIAL_DPAD_LEFT) result |= DPAD_LEFT;
    if (serialDpad & SERIAL_DPAD_RIGHT) result |= DPAD_RIGHT;
    return result;
}

bool serialButtonPressed(uint16_t mask) {
    return (serialPad.buttons & mask) != 0;
}

bool serialButtonRising(uint16_t mask) {
    return (serialPad.buttons & mask) != 0 &&
           (serialPreviousButtons & mask) == 0;
}

bool serialControlsNeutral() {
    return serialPad.leftX == 0.0f &&
           serialPad.leftY == 0.0f &&
           serialPad.rightX == 0.0f &&
           serialPad.rightY == 0.0f &&
           serialPad.buttons == 0 &&
           serialPad.dpad == 0;
}

void resetSerialPadState() {
    serialPad = {0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
    serialPreviousButtons = 0;
    serialPreviousDpad = 0;
    serialControlNeutralSinceMs = 0;
    serialTeachPressStartedMs = 0;
    serialTeachLongPressHandled = false;
}

void syncCommandToActualAfterSerialStop() {
    emergencyStopMotion();
    updateActualRobotPosition();

    if (jointPositionFinite(robot.actualJoints) &&
        cartesianPositionFinite(robot.actualTcp)) {
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
    }

    if (startupZHomeRequired) {
        syncStartupZHomeTargetToActual();
    }
}

void releaseSerialControl(const char* reason) {
    const bool wasArmed = serialControlArmed;

    if (robot.operatingMode == OperatingMode::AUTO &&
        (robot.autoState == AutoState::MOVE ||
         robot.autoState == AutoState::DWELL)) {
        pauseAuto(reason);
    } else if (robot.operatingMode == OperatingMode::MANUAL && wasArmed) {
        syncCommandToActualAfterSerialStop();
        if (reason != nullptr) Serial.println(reason);
    } else if (reason != nullptr) {
        Serial.println(reason);
    }

    serialControlArmed = false;
    serialControlSessionActive = false;
    resetSerialPadState();
}

bool serialControlOwnsInput() {
    if (!serialControlSessionActive) return false;
    return millis() - serialControlLastPacketMs <= SERIAL_CONTROL_TIMEOUT_MS;
}

void beginSerialControlSession() {
    const bool newSession = !serialControlSessionActive;
    serialControlSessionActive = true;
    serialControlLastPacketMs = millis();

    if (newSession) {
        // PL: PC przejmuje wejście wyłącznie po swojej procedurze uzbrojenia.
        //     Pad BT jest rozbrajany, aby dwa źródła nie były aktywne jednocześnie.
        // EN: The PC takes ownership only after its own arming procedure.
        //     Disarm the Bluetooth gamepad so two input sources cannot be active at once.
        // UA: ПК отримує керування лише після власної процедури активації.
        //     Bluetooth-геймпад вимикається, щоб два джерела не були активні одночасно.
        robot.gamepadArmed = false;
        gamepadNeutralSinceMs = 0;
        previousPadStart = false;
        previousPadX = false;
        serialControlArmed = false;
        resetSerialPadState();
        Serial.println("PC CONTROL: polaczono, pozostaw sterowanie neutralne");
    }

    Serial.println("PCACK,HELLO,1");
}

void handleSerialTeachCommand() {
    if (!serialControlSessionActive) {
        Serial.println("PCACK,TEACH,NEED_HELLO");
        return;
    }
    if (!serialControlArmed) {
        Serial.println("PCACK,TEACH,NOT_ARMED");
        return;
    }
    if (startupZHomeRequired) {
        Serial.println("PCACK,TEACH,HOME_REQUIRED");
        return;
    }
    if (robot.estopLatched) {
        Serial.println("PCACK,TEACH,ESTOP");
        return;
    }
    if (robot.operatingMode != OperatingMode::MANUAL) {
        Serial.println("PCACK,TEACH,NEED_MANUAL");
        return;
    }
    if (axesRunning()) {
        Serial.println("PCACK,TEACH,AXES_RUNNING");
        return;
    }

    const uint8_t before = programPointCount;
    teachCurrentPoint();

    if (programPointCount == static_cast<uint8_t>(before + 1)) {
        Serial.printf("PCACK,TEACH,OK,%u\n", programPointCount);
    } else {
        Serial.println("PCACK,TEACH,FAILED");
    }
}

void handleSerialClearCommand() {
    if (!serialControlSessionActive) {
        Serial.println("PCACK,CLEAR,NEED_HELLO");
        return;
    }
    if (!serialControlArmed) {
        Serial.println("PCACK,CLEAR,NOT_ARMED");
        return;
    }
    if (startupZHomeRequired) {
        Serial.println("PCACK,CLEAR,HOME_REQUIRED");
        return;
    }
    if (robot.estopLatched) {
        Serial.println("PCACK,CLEAR,ESTOP");
        return;
    }
    if (robot.operatingMode != OperatingMode::MANUAL) {
        Serial.println("PCACK,CLEAR,NEED_MANUAL");
        return;
    }
    if (axesRunning()) {
        Serial.println("PCACK,CLEAR,AXES_RUNNING");
        return;
    }

    clearProgram();

    if (programPointCount == 0) {
        Serial.println("PCACK,CLEAR,OK");
    } else {
        Serial.println("PCACK,CLEAR,FAILED");
    }
}

void parseSerialControlLine(const char* line) {
    if (strcmp(line, "PCHELLO") == 0) {
        beginSerialControlSession();
        return;
    }

    if (strcmp(line, "PCBYE") == 0) {
        if (serialControlSessionActive) {
            releaseSerialControl("PC CONTROL: rozlaczono");
        }
        Serial.println("PCACK,BYE,1");
        return;
    }

    if (strcmp(line, "PCTEACH") == 0) {
        handleSerialTeachCommand();
        return;
    }

    if (strcmp(line, "PCCLEAR") == 0) {
        handleSerialClearCommand();
        return;
    }

    int lx = 0;
    int ly = 0;
    int rx = 0;
    int ry = 0;
    unsigned int buttons = 0;
    unsigned int dpad = 0;

    if (sscanf(line, "PCPAD,%d,%d,%d,%d,%u,%u",
               &lx, &ly, &rx, &ry, &buttons, &dpad) == 6) {
        if (!serialControlSessionActive) {
            Serial.println("PCACK,NEED_HELLO,0");
            return;
        }

        serialPad.leftX = serialAxisFromInt(lx);
        serialPad.leftY = serialAxisFromInt(ly);
        serialPad.rightX = serialAxisFromInt(rx);
        serialPad.rightY = serialAxisFromInt(ry);
        serialPad.buttons = static_cast<uint16_t>(buttons & 0x3Fu);
        serialPad.dpad = static_cast<uint8_t>(dpad & 0x0Fu);
        serialControlLastPacketMs = millis();
        return;
    }
}

void serviceSerialControlRx() {
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());

        if (ch == '\r') continue;

        if (ch == '\n') {
            if (serialRxLength > 0) {
                serialRxLine[serialRxLength] = '\0';
                parseSerialControlLine(serialRxLine);
                serialRxLength = 0;
            }
            continue;
        }

        if (serialRxLength + 1 < SERIAL_RX_LINE_MAX) {
            serialRxLine[serialRxLength++] = ch;
        } else {
            serialRxLength = 0;
        }
    }
}

void serviceSerialControlSafety() {
    if (!serialControlSessionActive) return;

    if (millis() - serialControlLastPacketMs > SERIAL_CONTROL_TIMEOUT_MS) {
        releaseSerialControl("PC CONTROL TIMEOUT - ruch zatrzymany");
    }
}

void sendSerialControlStatus() {
    if (!serialControlSessionActive) return;

    const uint32_t now = millis();
    if (now - serialControlLastStatusMs < SERIAL_STATUS_PERIOD_MS) return;
    serialControlLastStatusMs = now;

    updateActualRobotPosition();

    Serial.printf(
        "PCSTATE,A1=%.2f,A2=%.2f,Z=%.2f,X=%.2f,Y=%.2f,ROT=%.1f,GRIP=%.1f,"
        "RELAY=%u,WORK=%s,CONTROL=%s,HOMEZ=%u,ARMED=%u,ESTOP=%u,POINTS=%u,SPEED=%u\n",
        robot.actualJoints.arm1Deg,
        robot.actualJoints.arm2Deg,
        robot.actualJoints.zMm,
        robot.actualTcp.xMm,
        robot.actualTcp.yMm,
        robot.toolRotateDeg,
        robot.gripperDeg,
        robot.relayOn ? 1u : 0u,
        robot.operatingMode == OperatingMode::AUTO ? "AUTO" : "MANUAL",
        robot.controlMode == ControlMode::TOOL ? "TOOL" : "JOINT",
        startupZHomeRequired ? 0u : 1u,
        serialControlArmed ? 1u : 0u,
        robot.estopLatched ? 1u : 0u,
        programPointCount,
        autoArmSpeedPercent);
}

bool updateSerialControlArming() {
    if (robot.operatingMode != OperatingMode::MANUAL) return false;
    if (serialControlArmed) return true;

    const uint32_t now = millis();

    if (!serialControlsNeutral()) {
        serialControlNeutralSinceMs = 0;
        return false;
    }

    if (serialControlNeutralSinceMs == 0) {
        serialControlNeutralSinceMs = now;
        return false;
    }

    if (now - serialControlNeutralSinceMs >= SERIAL_CONTROL_NEUTRAL_MS) {
        serialControlArmed = true;
        serialPreviousButtons = serialPad.buttons;
        serialPreviousDpad = serialPad.dpad;
        digitalWrite(PIN_LED_ERROR, LOW);
        Serial.println("PC CONTROL ARMED - MANUAL aktywny");
        return true;
    }

    return false;
}

void processSerialTeachButton() {
    const uint32_t now = millis();
    const bool teachPressed = serialButtonPressed(SERIAL_BTN_TEACH);
    const bool teachWasPressed = (serialPreviousButtons & SERIAL_BTN_TEACH) != 0;

    if (teachPressed && !teachWasPressed) {
        serialTeachPressStartedMs = now;
        serialTeachLongPressHandled = false;
    }

    if (teachPressed && serialTeachPressStartedMs != 0 &&
        !serialTeachLongPressHandled &&
        now - serialTeachPressStartedMs >= TEACH_LONG_PRESS_MS) {
        serialTeachLongPressHandled = true;
        clearProgram();
    }

    if (!teachPressed && teachWasPressed) {
        if (serialTeachPressStartedMs != 0 && !serialTeachLongPressHandled) {
            teachCurrentPoint();
        }
        serialTeachPressStartedMs = 0;
        serialTeachLongPressHandled = false;
    }
}

void processSerialControl(float deltaTimeSeconds) {
    if (!serialControlOwnsInput()) return;

    const uint8_t bluepadDpad = serialDpadToBluepad(serialPad.dpad);

    // PL: W AUTO aplikacja PC działa jak pad: START = start/pauza/wznowienie,
    //     D-pad = override prędkości, MODE = powrót do MANUAL.
    // EN: In AUTO the PC application behaves like the gamepad: START controls
    //     run/pause/resume, D-pad changes speed override, MODE returns to MANUAL.
    // UA: В AUTO застосунок ПК працює як геймпад: START керує пуском/паузою,
    //     D-pad змінює швидкість, MODE повертає до MANUAL.
    if (robot.operatingMode == OperatingMode::AUTO) {
        if (serialButtonRising(SERIAL_BTN_MODE) && !startupZHomeRequired) {
            toggleOperatingMode();
            serialPreviousButtons = serialPad.buttons;
            serialPreviousDpad = serialPad.dpad;
            return;
        }

        processAutoSpeedDpad(bluepadDpad);

        if (serialButtonRising(SERIAL_BTN_START)) {
            toggleAutoRunPause();
        }

        serialPreviousButtons = serialPad.buttons;
        serialPreviousDpad = serialPad.dpad;
        return;
    }

    previousAutoDpad = bluepadDpad;

    if (!updateSerialControlArming()) {
        serialPreviousButtons = serialPad.buttons;
        serialPreviousDpad = serialPad.dpad;
        return;
    }

    if (serialButtonRising(SERIAL_BTN_MODE) && !startupZHomeRequired) {
        toggleOperatingMode();
        serialPreviousButtons = serialPad.buttons;
        serialPreviousDpad = serialPad.dpad;
        return;
    }

    // PL: Procedura HOME Z działa identycznie jak z padem: prawy Y porusza Z,
    //     a jednoczesne A+B zatwierdza fizyczne zero.
    // EN: Startup Z HOME mirrors gamepad control: right Y jogs Z and A+B confirms zero.
    // UA: HOME Z працює так само, як з геймпада: правий Y рухає Z, A+B підтверджує нуль.
    if (startupZHomeRequired) {
        const bool wasRequired = startupZHomeRequired;
        const bool confirmZHome =
            serialButtonPressed(SERIAL_BTN_A) &&
            serialButtonPressed(SERIAL_BTN_B);

        processStartupZHome(serialPad.rightY, confirmZHome, deltaTimeSeconds);

        if (wasRequired && !startupZHomeRequired) {
            serialControlArmed = false;
            serialControlNeutralSinceMs = 0;
        }

        serialPreviousButtons = serialPad.buttons;
        serialPreviousDpad = serialPad.dpad;
        return;
    }

    if (serialButtonRising(SERIAL_BTN_START)) {
        toggleControlMode();
    }

    if (serialButtonRising(SERIAL_BTN_X)) {
        setRelayState(!robot.relayOn);
        Serial.printf("PC RELAY GPIO%d: %s\n",
                      PIN_RELAY, robot.relayOn ? "ON" : "OFF");
    }

    processSerialTeachButton();

    float gripCommand = 0.0f;
    if (serialButtonPressed(SERIAL_BTN_A)) gripCommand = 1.0f;
    if (serialButtonPressed(SERIAL_BTN_B)) gripCommand = -1.0f;

    if (robot.controlMode == ControlMode::JOINT) {
        updateServoTargets(serialPad.rightX, gripCommand, deltaTimeSeconds);
        processJointMode(serialPad.leftX, serialPad.leftY,
                         serialPad.rightY, deltaTimeSeconds);
    } else {
        updateServoTargets(0.0f, gripCommand, deltaTimeSeconds);
        processToolMode(serialPad.leftX, serialPad.leftY,
                        serialPad.rightY, serialPad.rightX,
                        deltaTimeSeconds);
    }

    serialPreviousButtons = serialPad.buttons;
    serialPreviousDpad = serialPad.dpad;
}
