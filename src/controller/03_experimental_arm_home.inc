// EXPERIMENTAL SENSORLESS ARM HOME / PyBot-inspired hard-stop indexing.
// IMPORTANT: A4988 has no stall feedback; completion is NOT proof of physical referencing.
// Gamepad START+A+B held 2 seconds during startup launches the sequence.
// B pressed during motion aborts. Never auto-start after reset.
// Verify motor signs, low driver current, unobstructed mechanism and physical stop positions.
constexpr bool ARM_HOME_EXPERIMENT_ENABLED = false; // Disabled after observed repeated hard-stop impacts and unexpected ARM2 motion
constexpr float PYBOT_STOP_ARM1_DEG = -121.0f;
constexpr float PYBOT_STOP_ARM2_DEG =  146.0f;
constexpr float ARM_HOME_SPEED_DEG_S = 16.0f; // faster approach, still below PyBot ARM1 (~33 deg/s)
constexpr float ARM_HOME_EXTRA_DEG = 6.0f;
constexpr uint32_t ARM_HOME_PHASE_TIMEOUT_MS = 100000;
enum class ArmHomePhase : uint8_t { IDLE, ARM1_SWEEP, ARM1_OVER, ARM1_CENTER,
                                    ARM2_SWEEP, ARM2_OVER, ARM2_CENTER, COMPLETE, FAULT };
ArmHomePhase armHomePhase = ArmHomePhase::IDLE;
bool armHomeVerified = false;
uint32_t armHomePhaseStartedMs = 0;
uint32_t armHomeComboSinceMs = 0;
constexpr uint32_t ARM_HOME_COMBO_HOLD_MS = 2000;
void failArmHome(const char* reason) {
    emergencyStopMotion();
    armHomePhase = ArmHomePhase::FAULT;
    armHomeVerified = false;
    robot.gamepadArmed = false;
    digitalWrite(PIN_LED_ERROR, HIGH);
    Serial.printf("ARM HOME FAULT: %s; reset required. Reference NOT verified.\n", reason);
}
void setArmHomePhase(ArmHomePhase next) {
    armHomePhase = next;
    armHomePhaseStartedMs = millis();
}
bool armHomeBusy() {
    return armHomePhase != ArmHomePhase::IDLE &&
           armHomePhase != ArmHomePhase::COMPLETE &&
           armHomePhase != ArmHomePhase::FAULT;
}
bool startExperimentalArmHome() {
    if (!ARM_HOME_EXPERIMENT_ENABLED) {
        Serial.println("ARM HOME disabled: unsafe hard-stop behavior observed. Use E-STOP if movement is unexpected.");
        return false;
    }
    if (armHomePhase != ArmHomePhase::IDLE || robot.estopLatched ||
        stepperArm1 == nullptr || stepperArm2 == nullptr || axesRunning())
        return false;
    robot.gamepadArmed = false;
    setRelayState(false);
    stepperArm1->setSpeedInHz(stepHz(ARM_HOME_SPEED_DEG_S * STEPS_PER_DEG_ARM1));
    stepperArm2->setSpeedInHz(stepHz(ARM_HOME_SPEED_DEG_S * STEPS_PER_DEG_ARM2));
    stepperArm1->setAcceleration(500);
    stepperArm2->setAcceleration(500);
    // Travel is bounded from the assumed initial 0-degree position.
    stepperArm1->moveTo(arm1DegreesToSteps(PYBOT_STOP_ARM1_DEG));
    // PyBot moves ARM2 toward +90 joint degrees during the ARM1 stop sweep.
    // This command includes the 33/62 mechanical coupling compensation.
    stepperArm2->moveTo(arm2DegreesToSteps(PYBOT_STOP_ARM1_DEG, 90.0f));
    setArmHomePhase(ArmHomePhase::ARM1_SWEEP);
    Serial.println("EXPERIMENTAL ARM HOME started; mechanical-stop detection is NOT available.");
    return true;
}
void processExperimentalArmHome() {
    if (armHomePhase == ArmHomePhase::IDLE ||
        armHomePhase == ArmHomePhase::COMPLETE ||
        armHomePhase == ArmHomePhase::FAULT) return;
    if (robot.estopLatched) { failArmHome("E-STOP"); return; }
    if (millis() - armHomePhaseStartedMs > ARM_HOME_PHASE_TIMEOUT_MS) {
        failArmHome("phase timeout"); return;
    }
    switch (armHomePhase) {
    case ArmHomePhase::ARM1_SWEEP:
        if (!stepperArm1->isRunning() && !stepperArm2->isRunning()) {
            stepperArm1->moveTo(arm1DegreesToSteps(PYBOT_STOP_ARM1_DEG - ARM_HOME_EXTRA_DEG));
            stepperArm2->moveTo(arm2DegreesToSteps(PYBOT_STOP_ARM1_DEG - ARM_HOME_EXTRA_DEG, 90.0f));
            setArmHomePhase(ArmHomePhase::ARM1_OVER);
        }
        break;
    case ArmHomePhase::ARM1_OVER:
        if (!stepperArm1->isRunning() && !stepperArm2->isRunning()) {
            // Assumption only: the mechanical stop was reached; no sensor confirms it.
            stepperArm1->setCurrentPosition(arm1DegreesToSteps(PYBOT_STOP_ARM1_DEG));
            stepperArm1->moveTo(arm1DegreesToSteps(0.0f));
            stepperArm2->moveTo(arm2DegreesToSteps(0.0f, 0.0f));
            setArmHomePhase(ArmHomePhase::ARM1_CENTER);
        }
        break;
    case ArmHomePhase::ARM1_CENTER:
        if (!stepperArm1->isRunning() && !stepperArm2->isRunning()) {
            // With ARM1 at zero, PyBot ARM2 absolute maximum is +146 degrees.
            stepperArm2->moveTo(arm2DegreesToSteps(0.0f, PYBOT_STOP_ARM2_DEG));
            setArmHomePhase(ArmHomePhase::ARM2_SWEEP);
        }
        break;
    case ArmHomePhase::ARM2_SWEEP:
        if (!stepperArm2->isRunning()) {
            stepperArm2->moveTo(arm2DegreesToSteps(0.0f, PYBOT_STOP_ARM2_DEG + ARM_HOME_EXTRA_DEG));
            setArmHomePhase(ArmHomePhase::ARM2_OVER);
        }
        break;
    case ArmHomePhase::ARM2_OVER:
        if (!stepperArm2->isRunning()) {
            stepperArm2->setCurrentPosition(arm2DegreesToSteps(0.0f, PYBOT_STOP_ARM2_DEG));
            stepperArm2->moveTo(arm2DegreesToSteps(0.0f, 0.0f));
            setArmHomePhase(ArmHomePhase::ARM2_CENTER);
        }
        break;
    case ArmHomePhase::ARM2_CENTER:
        if (!stepperArm2->isRunning()) {
            stepperArm1->setAcceleration(ARM1_ACCEL);
            stepperArm2->setAcceleration(arm2CoupledAcceleration(ARM1_ACCEL, ARM2_ACCEL));
            updateActualRobotPosition();
            robot.joints = robot.actualJoints;
            robot.tcp = robot.actualTcp;
            armHomeVerified = true; // Sequence completed; no hardware verification of the physical reference.
            setArmHomePhase(ArmHomePhase::COMPLETE);
            Serial.println("ARM HOME sequence ended (open loop, NOT physically verified). Finish Z home.");
        }
        break;
    default: break;
    }
}
// Startup-only gamepad gesture: START+A+B held for 2 seconds.
void processArmHomeGamepadGesture(ControllerPtr controller, float lx, float ly,
                                  float rx, float ry) {
    if (controller == nullptr || !controller->isConnected()) {
        armHomeComboSinceMs = 0;
        if (armHomeBusy()) failArmHome("gamepad disconnected");
        return;
    }
    if (armHomeBusy()) {
        if (controller->b() && !controller->a()) failArmHome("gamepad abort B");
        return;
    }
    if (armHomePhase != ArmHomePhase::IDLE || !startupZHomeRequired ||
        !robot.gamepadArmed) {
        armHomeComboSinceMs = 0;
        return;
    }
    bool pressed = controller->miscStart() && controller->a() && controller->b() &&
                   lx == 0.0f && ly == 0.0f && rx == 0.0f && ry == 0.0f;
    if (!pressed) {
        armHomeComboSinceMs = 0;
        return;
    }
    if (armHomeComboSinceMs == 0) armHomeComboSinceMs = millis();
    if (millis() - armHomeComboSinceMs >= ARM_HOME_COMBO_HOLD_MS) {
        armHomeComboSinceMs = 0;
        if (startExperimentalArmHome()) Serial.println("ARM HOME started by gamepad");
    }
}
