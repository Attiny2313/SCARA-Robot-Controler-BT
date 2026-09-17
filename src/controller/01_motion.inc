// ============================================================
// FUNKCJE POMOCNICZE / HELPER FUNCTIONS
// ============================================================
float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

uint32_t clampStepHz(float hz, uint32_t maximumHz) {
    if (hz < 1.0f) return 1;
    if (hz > static_cast<float>(maximumHz)) return maximumHz;
    return static_cast<uint32_t>(lroundf(hz));
}

float degToRad(float degrees) {
    return degrees * PI / 180.0f;
}

float radToDeg(float radians) {
    return radians * 180.0f / PI;
}

// PL: Normalizacja osi pada do zakresu -1...+1 z martwą strefą.
// EN: Normalize a gamepad axis to -1...+1 with a dead zone around center.
float normalizeJoystick(int32_t value) {
    if (abs(value) < JOYSTICK_DEADZONE) return 0.0f;
    constexpr float maximumAxisValue = 512.0f;
    return clampFloat(static_cast<float>(value) / maximumAxisValue, -1.0f, 1.0f);
}

int32_t arm1DegreesToSteps(float angleDeg) {
    return lroundf(angleDeg * STEPS_PER_DEG_ARM1 * SIGN_ARM1);
}

int32_t arm2DegreesToSteps(float angleDeg) {
    return lroundf(angleDeg * STEPS_PER_DEG_ARM2 * SIGN_ARM2);
}

int32_t zMillimetersToSteps(float zMm) {
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
    Serial.printf("AUTO SPEED ARM: %u%% (A1=%.1f deg/s, A2=%.1f deg/s)\n",
                  autoArmSpeedPercent,
                  AUTO_ARM1_SPEED_DEG_S * autoArmSpeedScale(),
                  AUTO_ARM2_SPEED_DEG_S * autoArmSpeedScale());
}

// PL: D-pad działa na zboczu: góra +10%, dół -10%, lewo = 100%.
//     Zmiana obowiązuje od następnego punktu PTP.
// EN: D-pad is edge-triggered: up +10%, down -10%, left = 100%.
//     The new value takes effect from the next PTP point.
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
// KINEMATYKA / KINEMATICS
// ============================================================
// PL: Kinematyka prosta: kąty obu ramion -> pozycja TCP X/Y/Z.
// EN: Forward kinematics: arm angles -> TCP position X/Y/Z.
CartesianPosition forwardKinematics(const JointPosition& joints) {
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
bool inverseKinematics(float xMm, float yMm, bool elbowUp, JointPosition& result) {
    const float radiusSquared = xMm * xMm + yMm * yMm;
    float cosTheta2 =
        (radiusSquared - ARM_LENGTH_1_MM * ARM_LENGTH_1_MM -
         ARM_LENGTH_2_MM * ARM_LENGTH_2_MM) /
        (2.0f * ARM_LENGTH_1_MM * ARM_LENGTH_2_MM);

    if (cosTheta2 < -1.0f || cosTheta2 > 1.0f) return false;

    cosTheta2 = clampFloat(cosTheta2, -1.0f, 1.0f);
    float sinTheta2 = sqrtf(1.0f - cosTheta2 * cosTheta2);
    if (!elbowUp) sinTheta2 = -sinTheta2;

    const float theta2 = atan2f(sinTheta2, cosTheta2);
    const float theta1 = atan2f(yMm, xMm) -
        atan2f(ARM_LENGTH_2_MM * sinTheta2,
               ARM_LENGTH_1_MM + ARM_LENGTH_2_MM * cosTheta2);

    result.arm1Deg = radToDeg(theta1);
    result.arm2Deg = radToDeg(theta2);
    result.zMm = robot.joints.zMm;

    return !(result.arm1Deg < ARM1_MIN_DEG || result.arm1Deg > ARM1_MAX_DEG ||
             result.arm2Deg < ARM2_MIN_DEG || result.arm2Deg > ARM2_MAX_DEG);
}

// ============================================================
// POZYCJA I RUCH / POSITION AND MOTION
// ============================================================
// PL: Pozycja rzeczywista jest wyliczana z liczników kroków FastAccelStepper.
// EN: Actual position is calculated from FastAccelStepper step counters.
void updateActualRobotPosition() {
    if (stepperArm1 == nullptr || stepperArm2 == nullptr || stepperZ == nullptr) return;

    robot.actualJoints.arm1Deg = arm1StepsToDegrees(stepperArm1->getCurrentPosition());
    robot.actualJoints.arm2Deg = arm2StepsToDegrees(stepperArm2->getCurrentPosition());
    robot.actualJoints.zMm = zStepsToMillimeters(stepperZ->getCurrentPosition());
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
    return {
        clampFloat(target.arm1Deg, ARM1_MIN_DEG, ARM1_MAX_DEG),
        clampFloat(target.arm2Deg, ARM2_MIN_DEG, ARM2_MAX_DEG),
        clampFloat(target.zMm, Z_MIN_MM, Z_MAX_MM)
    };
}

// PL: Natychmiastowe zatrzymanie generatorów kroków.
// EN: Immediately stop all step generators.
void emergencyStopMotion() {
    if (stepperArm1 != nullptr) stepperArm1->forceStop();
    if (stepperArm2 != nullptr) stepperArm2->forceStop();
    if (stepperZ != nullptr) stepperZ->forceStop();
}

// PL: Ruch ręczny przyjmuje małe, cyklicznie aktualizowane cele pozycji.
// EN: Manual motion receives small position targets updated every control cycle.
void commandManualJointPosition(const JointPosition& target, float deltaTimeSeconds) {
    if (robot.estopLatched || !robot.gamepadArmed ||
        robot.operatingMode != OperatingMode::MANUAL) return;

    const JointPosition clamped = clampJointPosition(target);

    if (deltaTimeSeconds > 0.0f) {
        const float arm1Delta = fabsf(clamped.arm1Deg - robot.joints.arm1Deg);
        const float arm2Delta = fabsf(clamped.arm2Deg - robot.joints.arm2Deg);
        const float zDelta = fabsf(clamped.zMm - robot.joints.zMm);

        const uint32_t arm1ManualMaxHz = clampStepHz(
            ARM1_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM1, ARM1_MAX_STEP_HZ);
        const uint32_t arm2ManualMaxHz = clampStepHz(
            ARM2_JOG_SPEED_DEG_S * STEPS_PER_DEG_ARM2, ARM2_MAX_STEP_HZ);
        const uint32_t zManualMaxHz = clampStepHz(
            Z_JOG_SPEED_MM_S * STEPS_PER_MM_Z, Z_MAX_STEP_HZ);

        if (arm1Delta > 0.0001f) {
            stepperArm1->setSpeedInHz(clampStepHz(
                arm1Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM1, arm1ManualMaxHz));
        }
        if (arm2Delta > 0.0001f) {
            stepperArm2->setSpeedInHz(clampStepHz(
                arm2Delta / deltaTimeSeconds * STEPS_PER_DEG_ARM2, arm2ManualMaxHz));
        }
        if (zDelta > 0.0001f) {
            stepperZ->setSpeedInHz(clampStepHz(
                zDelta / deltaTimeSeconds * STEPS_PER_MM_Z, zManualMaxHz));
        }
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

// PL: AUTO wykonuje ruch PTP. Prędkości osi są skalowane tak, aby w przybliżeniu
//     zakończyć ruch w tym samym czasie; przyspieszenia pozostają niezależne.
// EN: AUTO performs PTP motion. Axis speeds are scaled so the axes finish at
//     approximately the same time; acceleration profiles remain independent.
void commandAutoJointPosition(const JointPosition& target) {
    if (robot.estopLatched || robot.operatingMode != OperatingMode::AUTO) return;

    updateActualRobotPosition();
    const JointPosition clamped = clampJointPosition(target);

    const float d1 = fabsf(clamped.arm1Deg - robot.actualJoints.arm1Deg);
    const float d2 = fabsf(clamped.arm2Deg - robot.actualJoints.arm2Deg);
    const float dz = fabsf(clamped.zMm - robot.actualJoints.zMm);

    const float armScale = autoArmSpeedScale();
    const float arm1AutoSpeed = AUTO_ARM1_SPEED_DEG_S * armScale;
    const float arm2AutoSpeed = AUTO_ARM2_SPEED_DEG_S * armScale;

    const float t1 = d1 / arm1AutoSpeed;
    const float t2 = d2 / arm2AutoSpeed;
    const float tz = dz / AUTO_Z_SPEED_MM_S;
    const float moveTime = fmaxf(0.01f, fmaxf(t1, fmaxf(t2, tz)));

    if (d1 > 0.001f) {
        stepperArm1->setSpeedInHz(clampStepHz(
            d1 / moveTime * STEPS_PER_DEG_ARM1, ARM1_MAX_STEP_HZ));
    }
    if (d2 > 0.001f) {
        stepperArm2->setSpeedInHz(clampStepHz(
            d2 / moveTime * STEPS_PER_DEG_ARM2, ARM2_MAX_STEP_HZ));
    }
    if (dz > 0.001f) {
        stepperZ->setSpeedInHz(clampStepHz(
            dz / moveTime * STEPS_PER_MM_Z, Z_MAX_STEP_HZ));
    }

    stepperArm1->moveTo(arm1DegreesToSteps(clamped.arm1Deg));
    stepperArm2->moveTo(arm2DegreesToSteps(clamped.arm2Deg));
    stepperZ->moveTo(zMillimetersToSteps(clamped.zMm));

    robot.joints = clamped;
    robot.tcp = forwardKinematics(robot.joints);
}

