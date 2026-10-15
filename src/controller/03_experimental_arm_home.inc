// EXPERIMENTAL SENSORLESS ARM HOME / PyBot-inspired hard-stop indexing.
// IMPORTANT: A4988 has no stall feedback; completion is NOT proof of physical referencing.
// Gamepad START+A+B held 2 seconds during startup launches the sequence.
// B pressed during motion aborts. Never auto-start after reset.
// Verify motor signs, low driver current, unobstructed mechanism and physical stop positions.
// Experimental open-loop reference: the A4988 does not detect physical contact.
// Do NOT introduce a second stop-overrun. Contact is assumed, never measured.
constexpr bool ARM_HOME_EXPERIMENT_ENABLED = false; // Paused: physical axis mapping/coupling must be verified
constexpr float PYBOT_STOP_ARM1_DEG = -121.0f;
constexpr float PYBOT_STOP_ARM2_DEG = 146.0f;
constexpr float ARM_HOME_SPEED_DEG_S = 6.0f;
constexpr uint32_t ARM_HOME_PHASE_TIMEOUT_MS = 45000;
constexpr uint32_t ARM_HOME_COMBO_HOLD_MS = 2000;
enum class ArmHomePhase : uint8_t {
    IDLE, ARM1_MINUS, ARM2_PLUS, RETURN_ZERO, COMPLETE, FAULT
};
ArmHomePhase armHomePhase = ArmHomePhase::IDLE;
bool armHomeVerified = false; // sequence finished; NOT a measured reference
uint32_t armHomePhaseStartedMs = 0;
uint32_t armHomeComboSinceMs = 0;

void failArmHome(const char* reason) {
    emergencyStopMotion();
    armHomePhase = ArmHomePhase::FAULT;
    armHomeVerified = false;
    robot.gamepadArmed = false;
    digitalWrite(PIN_LED_ERROR, HIGH);
    Serial.printf("ARM HOME FAULT: %s; reset required.\\n", reason);
}
void setArmHomePhase(ArmHomePhase next) {
    armHomePhase = next;
    armHomePhaseStartedMs = millis();
}
bool armHomeBusy() {
    return armHomePhase == ArmHomePhase::ARM1_MINUS ||
           armHomePhase == ArmHomePhase::ARM2_PLUS ||
           armHomePhase == ArmHomePhase::RETURN_ZERO;
}
bool startExperimentalArmHome() {
    if (!ARM_HOME_EXPERIMENT_ENABLED || armHomePhase != ArmHomePhase::IDLE ||
        robot.estopLatched || stepperArm1 == nullptr ||
        stepperArm2 == nullptr || axesRunning()) return false;
    robot.gamepadArmed = false;
    setRelayState(false);
    stepperArm1->setSpeedInHz(stepHz(ARM_HOME_SPEED_DEG_S * STEPS_PER_DEG_ARM1));
    stepperArm2->setSpeedInHz(stepHz(ARM_HOME_SPEED_DEG_S * STEPS_PER_DEG_ARM2));
    stepperArm1->setAcceleration(500);
    stepperArm2->setAcceleration(500);
    // ONLY ARM1 is driven in phase one; ARM2 motor holds its position.
    // Its mechanical joint angle may nevertheless change due to 33/62 coupling.
    stepperArm1->moveTo(arm1DegreesToSteps(PYBOT_STOP_ARM1_DEG));
    setArmHomePhase(ArmHomePhase::ARM1_MINUS);
    Serial.println("ARM HOME: phase 1 ARM1 -> -121, ARM2 motor stationary");
    return true;
}
void processExperimentalArmHome() {
    if (!armHomeBusy()) return;
    if (robot.estopLatched) { failArmHome("E-STOP"); return; }
    if (millis() - armHomePhaseStartedMs > ARM_HOME_PHASE_TIMEOUT_MS) {
        failArmHome("phase timeout"); return;
    }
    switch (armHomePhase) {
    case ArmHomePhase::ARM1_MINUS:
        if (!stepperArm1->isRunning()) {
            // ASSUMED hard stop at -121 (no stall feedback).
            // Set the counter only after phase one, before referencing ARM2.
            stepperArm1->setCurrentPosition(arm1DegreesToSteps(PYBOT_STOP_ARM1_DEG));
            // At ARM1=-121 the motor coordinate is A2 + A1*(33/62).
            stepperArm2->moveTo(arm2DegreesToSteps(PYBOT_STOP_ARM1_DEG, PYBOT_STOP_ARM2_DEG));
            setArmHomePhase(ArmHomePhase::ARM2_PLUS);
            Serial.println("ARM HOME: phase 2 ARM2 -> +146, ARM1 motor stationary");
        }
        break;
    case ArmHomePhase::ARM2_PLUS:
        if (!stepperArm2->isRunning()) {
            // ASSUMED hard stop at +146 (no stall feedback).
            stepperArm2->setCurrentPosition(
                arm2DegreesToSteps(PYBOT_STOP_ARM1_DEG, PYBOT_STOP_ARM2_DEG));
            // Explicitly give both motors their compensated 0/0 targets.
            stepperArm1->moveTo(arm1DegreesToSteps(0.0f));
            stepperArm2->moveTo(arm2DegreesToSteps(0.0f, 0.0f));
            setArmHomePhase(ArmHomePhase::RETURN_ZERO);
            Serial.println("ARM HOME: phase 3 ARM1/ARM2 -> 0/0");
        }
        break;
    case ArmHomePhase::RETURN_ZERO:
        if (!stepperArm1->isRunning() && !stepperArm2->isRunning()) {
            stepperArm1->setAcceleration(ARM1_ACCEL);
            stepperArm2->setAcceleration(arm2CoupledAcceleration(ARM1_ACCEL, ARM2_ACCEL));
            updateActualRobotPosition();
            robot.joints = robot.actualJoints;
            robot.tcp = robot.actualTcp;
            armHomeVerified = true; // open-loop completion only
            setArmHomePhase(ArmHomePhase::COMPLETE);
            Serial.println("ARM HOME ended; positions ASSUMED, not independently detected. Home Z next.");
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
