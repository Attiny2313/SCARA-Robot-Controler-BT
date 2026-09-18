// ============================================================
// PL: FUNKCJE POMOCNICZE
// EN: HELPER FUNCTIONS
// UA: ДОПОМІЖНІ ФУНКЦІЇ
// ============================================================
float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

// PL: Konwersja prędkości na częstotliwość STEP bez górnego limitu.
//     Prędkość wynika z zadanych wartości osi oraz procentu AUTO.
// EN: Convert speed to STEP frequency without an upper software limit.
// UA: Перетворення швидкості на частоту STEP без верхнього програмного обмеження.
//     Швидкість визначається заданими значеннями осей та відсотком AUTO.
uint32_t stepHz(float hz) {
    if (!isfinite(hz) || hz < 1.0f) return 1;
    return static_cast<uint32_t>(lroundf(hz));
}

bool jointPositionFinite(const JointPosition& p) {
    return isfinite(p.arm1Deg) && isfinite(p.arm2Deg) && isfinite(p.zMm);
}

bool cartesianPositionFinite(const CartesianPosition& p) {
    return isfinite(p.xMm) && isfinite(p.yMm) && isfinite(p.zMm);
}

float degToRad(float degrees) {
    return degrees * PI / 180.0f;
}

float radToDeg(float radians) {
    return radians * 180.0f / PI;
}

// PL: Normalizacja osi pada do zakresu -1...+1 z martwą strefą.
// EN: Normalize a gamepad axis to -1...+1 with a dead zone around center.
// UA: Нормалізація осі геймпада до діапазону -1...+1 з мертвою зоною біля центру.
float normalizeJoystick(int32_t value) {
    if (abs(value) < JOYSTICK_DEADZONE) return 0.0f;
    constexpr float maximumAxisValue = 512.0f;
    return clampFloat(static_cast<float>(value) / maximumAxisValue, -1.0f, 1.0f);
}

int32_t arm1DegreesToSteps(float angleDeg) {
    if (!isfinite(angleDeg)) return 0;
    return lroundf(angleDeg * STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

int32_t arm2DegreesToSteps(float angleDeg) {
    if (!isfinite(angleDeg)) return 0;
    return lroundf(angleDeg * STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

int32_t zMillimetersToSteps(float zMm) {
    if (!isfinite(zMm)) return 0;
    return lroundf(zMm * STEPS_PER_MM_Z * SIGN_Z);
}

float arm1StepsToDegrees(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

float arm2StepsToDegrees(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

float zStepsToMillimeters(int32_t steps) {
    return static_cast<float>(steps) / (STEPS_PER_MM_Z * SIGN_Z);
}

float autoArmSpeedScale() {
    return static_cast<float>(autoArmSpeedPercent) / 100.0f;
}

void setAutoArmSpeedPercent(int32_t requestedPercent) {
    const uint16_t clampedPercent = static_cast<uint16_t>(constrain(
        requestedPercent,
        static_cast<int32_t>(AUTO_ARM_SPEED_PERCENT_MIN),
        static_cast<int32_t>(AUTO_ARM_SPEED_PERCENT_MAX)));

    if (clampedPercent == autoArmSpeedPercent) return;

    autoArmSpeedPercent = clampedPercent;
    const float scale = autoArmSpeedScale();
    Serial.printf("AUTO SPEED: %u%% (A1=%.1f deg/s, A2=%.1f deg/s, Z=%.2f mm/s)\n",
                  autoArmSpeedPercent,
                  AUTO_ARM1_SPEED_DEG_S * scale,
                  AUTO_ARM2_SPEED_DEG_S * scale,
                  AUTO_Z_SPEED_MM_S * scale);
}

// PL: D-pad działa na zboczu: góra +10%, dół -10%, lewo = 100%.
//     Zmiana obowiązuje od następnego punktu PTP i skaluje wszystkie 3 osie.
// EN: D-pad is edge-triggered: up +10%, down -10%, left = 100%.
//     The new value takes effect from the next PTP point and scales all 3 axes.
// UA: D-pad обробляється за фронтом: вгору +10%, вниз -10%, вліво = 100%.
//     Нова величина діє від наступної точки PTP і масштабує всі 3 осі.
void processAutoSpeedDpad(uint8_t dpad) {
    const bool upPressed = (dpad & DPAD_UP) != 0 && (previousAutoDpad & DPAD_UP) == 0;
    const bool downPressed = (dpad & DPAD_DOWN) != 0 && (previousAutoDpad & DPAD_DOWN) == 0;
    const bool leftPressed = (dpad & DPAD_LEFT) != 0 && (previousAutoDpad & DPAD_LEFT) == 0;

    if (leftPressed) {
        setAutoArmSpeedPercent(AUTO_ARM_SPEED_PERCENT_DEFAULT);
    } else if (upPressed) {
        setAutoArmSpeedPercent(static_cast<int32_t>(autoArmSpeedPercent) + AUTO_ARM_SPEED_PERCENT_STEP);
    } else if (downPressed) {
        setAutoArmSpeedPercent(static_cast<int32_t>(autoArmSpeedPercent) - AUTO_ARM_SPEED_PERCENT_STEP);
    }

    previousAutoDpad = dpad;
}

// ============================================================
// PL: KINEMATYKA
// EN: KINEMATICS
// UA: КІНЕМАТИКА
// ============================================================
// PL: Kinematyka prosta: kąty obu ramion -> pozycja TCP X/Y/Z.
// EN: Forward kinematics: arm angles -> TCP position X/Y/Z.
// UA: Пряма кінематика: кути обох ланок -> положення TCP X/Y/Z.
CartesianPosition forwardKinematics(const JointPosition& joints) {
    if (!jointPositionFinite(joints)) return {NAN, NAN, NAN};

    const float theta1 = degToRad(joints.arm1Deg);
    const float theta2 = degToRad(joints.arm2Deg);

    CartesianPosition result;
    result.xMm = ARM_LENGTH_1_MM * cosf(theta1) +
                 ARM_LENGTH_2_MM * cosf(theta1 + theta2);
    result.yMm = ARM_LENGTH_1_MM * sinf(theta1) +
                 ARM_LENGTH_2_MM * sinf(theta1 + theta2);
    result.zMm = joints.zMm;
    return result;
}

// PL: Kinematyka odwrotna dla płaszczyzny XY. Parametr elbowUp wybiera
//     jedną z dwóch geometrycznie możliwych konfiguracji ramienia.
// EN: Inverse kinematics for the XY plane. elbowUp selects one of the two
//     geometrically possible arm configurations.
// UA: Обернена кінематика для площини XY. Параметр elbowUp вибирає одну
//     з двох геометрично можливих конфігурацій маніпулятора.
bool inverseKinematics(float xMm, float yMm, bool elbowUp, JointPosition& result) {
    if (!isfinite(xMm) || !isfinite(yMm)) return false;

    const float radiusSquared = xMm * xMm + yMm * yMm;
    float cosTheta2 =
        (radiusSquared - ARM_LENGTH_1_MM * ARM_LENGTH_1_MM -
         ARM_LENGTH_2_MM * ARM_LENGTH_2_MM) /
        (2.0f * ARM_LENGTH_1_MM * ARM_LENGTH_2_MM);

    if (!isfinite(cosTheta2) || cosTheta2 < -1.0f || cosTheta2 > 1.0f) return false;

    cosTheta2 = clampFloat(cosTheta2, -1.0f, 1.0f);
    float sinTheta2 = sqrtf(fmaxf(0.0f, 1.0f - cosTheta2 * cosTheta2));
    if (!elbowUp) sinTheta2 = -sinTheta2;

    const float theta2 = atan2f(sinTheta2, cosTheta2);
    const float theta1 = atan2f(yMm, xMm) -
        atan2f(ARM_LENGTH_2_MM * sinTheta2,
               ARM_LENGTH_1_MM + ARM_LENGTH_2_MM * cosTheta2);

    result.arm1Deg = radToDeg(theta1);
    result.arm2Deg = radToDeg(theta2);
    result.zMm = robot.joints.zMm;

    return jointPositionFinite(result) &&
           !(result.arm1Deg < ARM1_MIN_DEG || result.arm1Deg > ARM1_MAX_DEG ||
             result.arm2Deg < ARM2_MIN_DEG || result.arm2Deg > ARM2_MAX_DEG);
}

// ============================================================
// PL: POZYCJA I RUCH
// EN: POSITION AND MOTION
// UA: ПОЛОЖЕННЯ ТА РУХ
// ============================================================
// PL: Pozycja rzeczywista jest wyliczana z liczników kroków FastAccelStepper.
// EN: Actual position is calculated from FastAccelStepper step counters.
// UA: Фактичне положення обчислюється з лічильників кроків FastAccelStepper.
void updateActualRobotPosition() {
    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return;

    JointPosition measured;
    measured.arm1Deg = arm1StepsToDegrees(stepperArm1->getCurrentPosition());
    measured.arm2Deg = arm2StepsToDegrees(stepperArm2->getCurrentPosition());
    measured.zMm = zStepsToMillimeters(stepperZ->getCurrentPosition());

    if (!jointPositionFinite(measured)) {
        Serial.printf("NUMERIC ERROR ACT: raw A1=%ld A2=%ld Z=%ld\n",
                      static_cast<long>(stepperArm1->getCurrentPosition()),
                      static_cast<long>(stepperArm2->getCurrentPosition()),
                      static_cast<long>(stepperZ->getCurrentPosition()));
        return;
    }

    robot.actualJoints = measured;
    robot.actualTcp = forwardKinematics(robot.actualJoints);
}

bool axesRunning() {
    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return false;
    return stepperArm1->isRunning() || stepperArm2->isRunning() || stepperZ->isRunning();
}

bool motionFinished() {
    return !axesRunning();
}

JointPosition clampJointPosition(const JointPosition& target) {
    if (!jointPositionFinite(target)) return robot.actualJoints;
    return {
        clampFloat(target.arm1Deg, ARM1_MIN_DEG, ARM1_MAX_DEG),
        clampFloat(target.arm2Deg, ARM2_MIN_DEG, ARM2_MAX_DEG),
        clampFloat(target.zMm, Z_MIN_MM, Z_MAX_MM)
    };
}

// PL: Natychmiastowe zatrzymanie generatorów kroków.
// EN: Immediately stop all step generators.
// UA: Негайна зупинка всіх генераторів кроків.
void emergencyStopMotion() {
    if (stepperArm1 != nullptr) stepperArm1->forceStop();
    if (stepperArm2 != nullptr) stepperArm2->forceStop();
    if (stepperZ != nullptr) stepperZ->forceStop();
}

// PL: Ruch ręczny przyjmuje małe, cyklicznie aktualizowane cele pozycji.
// EN: Manual motion receives small position targets updated every control cycle.
// UA: Ручний рух отримує невеликі цільові зміщення, що оновлюються в кожному циклі керування.
void commandManualJointPosition(const JointPosition& target, float deltaTimeSeconds) {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL) return;

    if (!jointPositionFinite(target)) {
        updateActualRobotPosition();
        if (jointPositionFinite(robot.actualJoints)) {
            robot.joints = robot.actualJoints;
            robot.tcp = robot.actualTcp;
        }
        Serial.println("NUMERIC ERROR MANUAL: target NaN/Inf, resync z pozycji krokow");
        return;
    }

    const JointPosition clamped = clampJointPosition(target);

    if (deltaTimeSeconds > 0.0f) {
        const float arm1Delta = fabsf(clamped.arm1Deg - robot.joints.arm1Deg);
        const float arm2Delta = fabsf(clamped.arm2Deg - robot.joints.arm2Deg);
        const float zDelta = fabsf(clamped.zMm - robot.joints.zMm);

        const uint32_t arm1ManualMaxHz = stepHz(
            ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1);
        const uint32_t arm2ManualMaxHz = stepHz(
            ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2);
        const uint32_t zManualMaxHz = stepHz(
            Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z);

        if (arm1Delta > 0.0001f) {
            stepperArm1->setSpeedInHz(stepHz(fminf(
                arm1Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM1,
                static_cast<float>(arm1ManualMaxHz))));
        }
        if (arm2Delta > 0.0001f) {
            stepperArm2->setSpeedInHz(stepHz(fminf(
                arm2Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM2,
                static_cast<float>(arm2ManualMaxHz))));
        }
        if (zDelta > 0.0001f) {
            stepperZ->setSpeedInHz(stepHz(fminf(
                zDelta / deltaTimeSeconds * STEPS_PER_MM_Z,
                static_cast<float>(zManualMaxHz))));
        }
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

// PL: AUTO wykonuje ruch PTP. Ten sam procent prędkości skaluje ARM1, ARM2 i Z.
//     Brak dodatkowych programowych limitów częstotliwości STEP.
// EN: AUTO performs PTP motion. The same speed percentage scales ARM1, ARM2 and Z.
//     There are no additional software STEP-frequency caps.
// UA: AUTO виконує рух PTP. Той самий відсоток швидкості масштабує ARM1, ARM2 і Z.
//     Додаткові програмні обмеження частоти STEP не застосовуються.
void commandAutoJointPosition(const JointPosition& target) {
    if (robot.estopLatched || robot.operatingMode != OperatingMode::AUTO) return;

    if (!jointPositionFinite(target)) {
        Serial.println("NUMERIC ERROR AUTO: punkt zawiera NaN/Inf");
        return;
    }

    updateActualRobotPosition();
    if (!jointPositionFinite(robot.actualJoints)) return;

    const JointPosition clamped = clampJointPosition(target);

    const float d1 = fabsf(clamped.arm1Deg - robot.actualJoints.arm1Deg);
    const float d2 = fabsf(clamped.arm2Deg - robot.actualJoints.arm2Deg);
    const float dz = fabsf(clamped.zMm - robot.actualJoints.zMm);

    const float speedScale = autoArmSpeedScale();
    const float arm1AutoSpeed = AUTO_ARM1_SPEED_DEG_S * speedScale;
    const float arm2AutoSpeed = AUTO_ARM2_SPEED_DEG_S * speedScale;
    const float zAutoSpeed = AUTO_Z_SPEED_MM_S * speedScale;

    const float t1 = d1 / arm1AutoSpeed;
    const float t2 = d2 / arm2AutoSpeed;
    const float tz = dz / zAutoSpeed;
    const float moveTime = fmaxf(0.01f, fmaxf(t1, fmaxf(t2, tz)));

    if (!isfinite(moveTime)) {
        Serial.println("NUMERIC ERROR AUTO: nieprawidlowy czas ruchu");
        return;
    }

    if (d1 > 0.001f) {
        stepperArm1->setSpeedInHz(stepHz(
            d1 / moveTime * STEPS_PER_DEG_ARM1));
    }
    if (d2 > 0.001f) {
        stepperArm2->setSpeedInHz(stepHz(
            d2 / moveTime * STEPS_PER_DEG_ARM2));
    }
    if (dz > 0.001f) {
        stepperZ->setSpeedInHz(stepHz(
            dz / moveTime * STEPS_PER_MM_Z));
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}
