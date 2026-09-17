#include "logo_pp_oled.h"

constexpr uint32_t OLED_SPLASH_DURATION_MS = 5000;
uint32_t oledSplashUntilMs = 0;

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
// UA: Простий фільтр першого порядку згладжує різкі зміни заданого положення сервоприводів.
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
// WYŚWIETLACZ OLED I2C / I2C OLED DISPLAY
// ============================================================
uint8_t detectOledAddress() {
    const uint8_t addresses[] = {OLED_ADDRESS_PRIMARY, OLED_ADDRESS_SECONDARY};
    for (uint8_t address : addresses) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) return address;
    }
    return 0;
}

bool initializeDisplay() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    displayI2cAddress = detectOledAddress();

    if (displayI2cAddress == 0) {
        Serial.println("OLED: nie znaleziono urządzenia pod 0x3C/0x3D");
        return false;
    }

    if (!display.begin(SSD1306_SWITCHCAPVCC, displayI2cAddress)) {
        Serial.printf("OLED: inicjalizacja SSD1306 nieudana (0x%02X)\n", displayI2cAddress);
        return false;
    }

    // PL: Po starcie wyświetl logo PP przez 5 sekund bez blokowania programu.
    // EN: Show the PP logo for 5 seconds after startup without blocking the program.
    // UA: Після запуску показувати логотип PP протягом 5 секунд без блокування програми.
    display.clearDisplay();
    display.drawBitmap(
        (OLED_WIDTH - PP_LOGO_WIDTH) / 2,
        (OLED_HEIGHT - PP_LOGO_HEIGHT) / 2,
        PP_LOGO_BITMAP,
        PP_LOGO_WIDTH,
        PP_LOGO_HEIGHT,
        SSD1306_WHITE);
    display.display();

    oledSplashUntilMs = millis() + OLED_SPLASH_DURATION_MS;

    Serial.printf("OLED: SSD1306 128x64 znaleziony pod 0x%02X\n", displayI2cAddress);
    return true;
}

const char* autoStateShortName() {
    switch (robot.autoState) {
        case AutoState::IDLE: return "IDLE";
        case AutoState::MOVE: return "RUN";
        case AutoState::DWELL: return "WAIT";
        case AutoState::PAUSED: return "PAUSE";
        case AutoState::FINISHED: return "DONE";
        case AutoState::FAULT: return "FAULT";
    }
    return "?";
}

void oledPrintFloatOrErr(float value, uint8_t decimals) {
    if (isfinite(value)) {
        display.print(value, decimals);
    } else {
        display.print("ERR");
    }
}

void refreshDisplay() {
    if (!displayAvailable) return;

    const uint32_t now = millis();

    // PL: Splash nie blokuje pętli głównej; przez 5 s tylko nie nadpisujemy OLED.
    // EN: The splash does not block the main loop; for 5 s we only avoid redrawing OLED.
    // UA: Splash не блокує головний цикл; протягом 5 с OLED просто не перемальовується.
    if (oledSplashUntilMs != 0) {
        if (static_cast<int32_t>(now - oledSplashUntilMs) < 0) return;
        oledSplashUntilMs = 0;
    }

    if (now - lastDisplayMs < DISPLAY_PERIOD_MS) return;
    lastDisplayMs = now;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    const bool numericOk = jointPositionFinite(robot.actualJoints) &&
                           cartesianPositionFinite(robot.actualTcp) &&
                           jointPositionFinite(robot.joints);

    display.setCursor(0, 0);
    if (!numericOk) {
        display.print("SCARA  NUM ERR");
    } else if (robot.estopLatched) {
        display.print("SCARA  E-STOP");
    } else if (robot.operatingMode == OperatingMode::AUTO) {
        display.print("AUTO  ");
        display.print(autoStateShortName());
    } else {
        display.print("MANUAL  ");
        display.print(robot.controlMode == ControlMode::TOOL ? "TOOL" : "JOINT");
    }

    display.setCursor(0, 10);
    display.print("BT:");
    display.print(robot.controllerConnected ? "OK" : "WAIT");
    display.print(" T:");
    display.print(programPointCount);
    if (robot.operatingMode == OperatingMode::AUTO) {
        display.print(" S:");
        display.print(autoArmSpeedPercent);
        display.print('%');
    }

    display.setCursor(0, 20);
    display.print("A1:");
    oledPrintFloatOrErr(robot.actualJoints.arm1Deg, 1);
    display.print(" deg");

    display.setCursor(0, 30);
    display.print("A2:");
    oledPrintFloatOrErr(robot.actualJoints.arm2Deg, 1);
    display.print(" deg");

    display.setCursor(0, 40);
    display.print("Z :");
    oledPrintFloatOrErr(robot.actualJoints.zMm, 1);
    display.print(" mm");

    display.setCursor(0, 50);
    if (!numericOk) {
        display.print("Sprawdz Serial");
    } else if (robot.operatingMode == OperatingMode::AUTO && programPointCount > 0) {
        display.print("P:");
        display.print(currentProgramPoint + 1);
        display.print('/');
        display.print(programPointCount);
        display.print(" ROT:");
        oledPrintFloatOrErr(robot.toolRotateDeg, 0);
    } else {
        display.print("X:");
        oledPrintFloatOrErr(robot.actualTcp.xMm, 0);
        display.print(" Y:");
        oledPrintFloatOrErr(robot.actualTcp.yMm, 0);
    }

    display.display();
}
