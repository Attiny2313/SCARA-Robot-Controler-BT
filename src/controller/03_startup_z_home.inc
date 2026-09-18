// ============================================================
// STARTOWE USTAWIANIE HOME Z / STARTUP Z HOME ASSIST
// ============================================================
// PL: ARM1 i ARM2 są ustawiane ręcznie w pozycji 0 przed resetem.
//     Po uruchomieniu operator ustawia oś Z padem, a następnie zatwierdza
//     pozycję HOME kombinacją A+B. Dopiero wtedy włączane jest normalne sterowanie.
// EN: ARM1 and ARM2 are manually placed at 0 before reset. After startup,
//     the operator jogs Z with the gamepad and confirms HOME with A+B.
// UA: ARM1 і ARM2 вручну встановлюються в 0 перед скиданням. Після запуску
//     оператор переміщує Z геймпадом і підтверджує HOME комбінацією A+B.

constexpr float STARTUP_Z_HOME_JOG_SPEED_MM_S = 1.5f;
constexpr float STARTUP_Z_HOME_RELATIVE_LIMIT_MM = 320.0f;

bool startupZHomeRequired = true;
float startupZHomeTargetMm = 0.0f;

void syncStartupZHomeTargetToActual() {
    if (stepperZ == nullptr) {
        startupZHomeTargetMm = 0.0f;
        return;
    }

    startupZHomeTargetMm = zStepsToMillimeters(stepperZ->getCurrentPosition());
}

void beginStartupZHome() {
    startupZHomeRequired = true;
    syncStartupZHomeTargetToActual();

    Serial.println("STARTUP HOME Z: prawy analog Y = ruch Z");
    Serial.println("STARTUP HOME Z: ustaw fizyczne Z=0, zwolnij analog i nacisnij A+B");
}

void completeStartupZHome() {
    if (stepperZ == nullptr || stepperZ->isRunning()) return;

    // PL: Dopiero po potwierdzeniu operatora bieżące położenie mechaniczne Z
    //     staje się programowym HOME_Z_MM.
    // EN: Only after operator confirmation does the current mechanical Z
    //     position become software HOME_Z_MM.
    // UA: Лише після підтвердження оператора поточне механічне положення Z
    //     стає програмним HOME_Z_MM.
    stepperZ->setCurrentPosition(zMillimetersToSteps(HOME_Z_MM));
    startupZHomeTargetMm = HOME_Z_MM;

    updateActualRobotPosition();
    if (jointPositionFinite(robot.actualJoints) && cartesianPositionFinite(robot.actualTcp)) {
        robot.joints = robot.actualJoints;
        robot.tcp = robot.actualTcp;
    } else {
        robot.joints.zMm = HOME_Z_MM;
        robot.actualJoints.zMm = HOME_Z_MM;
        robot.tcp = forwardKinematics(robot.joints);
        robot.actualTcp = robot.tcp;
    }

    startupZHomeRequired = false;

    // PL: Po A+B pad jest ponownie rozbrajany. Operator musi puścić przyciski
    //     i pozostawić pad neutralny przez standardowy czas uzbrajania.
    robot.gamepadArmed = false;
    gamepadNeutralSinceMs = 0;
    digitalWrite(PIN_LED_ERROR, LOW);

    Serial.println("STARTUP HOME Z: zatwierdzone, Z = 0.00 mm");
    Serial.println("MANUAL: pusc A+B i pozostaw pad neutralny, aby uzbroic sterowanie");
}

void processStartupZHome(float axisZ, bool confirmCombo, float deltaTimeSeconds) {
    if (!startupZHomeRequired || stepperZ == nullptr || robot.estopLatched) return;

    // PL: W czasie ustawiania HOME Z nie stosujemy normalnych limitów Z,
    //     bo pozycja absolutna nie jest jeszcze znana. Ograniczamy jedynie
    //     maksymalny ruch wzgledem punktu startowego.
    // EN: Normal Z limits are not used before HOME is known; only a relative
    //     startup travel guard is applied.
    if (axisZ != 0.0f && deltaTimeSeconds > 0.0f) {
        startupZHomeTargetMm +=
            -axisZ * STARTUP_Z_HOME_JOG_SPEED_MM_S * deltaTimeSeconds;
        startupZHomeTargetMm = clampFloat(
            startupZHomeTargetMm,
            -STARTUP_Z_HOME_RELATIVE_LIMIT_MM,
            STARTUP_Z_HOME_RELATIVE_LIMIT_MM);

        const float requestedSpeedMmS =
            fmaxf(0.15f, fabsf(axisZ)) * STARTUP_Z_HOME_JOG_SPEED_MM_S;
        stepperZ->setSpeedInHz(stepHz(requestedSpeedMmS * STEPS_PER_MM_Z));
        stepperZ->moveTo(zMillimetersToSteps(startupZHomeTargetMm));
    } else if (stepperZ->isRunning()) {
        // PL: W tym specjalnym trybie po puszczeniu analogu zatrzymujemy Z od razu,
        //     żeby operator mógł precyzyjnie ustawić fizyczne zero bez dojazdu
        //     do wcześniej narastającego celu.
        // EN: In startup homing, releasing the stick stops Z immediately for
        //     precise manual zero positioning.
        stepperZ->forceStop();
        syncStartupZHomeTargetToActual();
    }

    // PL: A+B zatwierdza HOME tylko przy puszczonym analogu i zatrzymanej osi.
    // EN: A+B confirms HOME only with the stick released and Z fully stopped.
    // UA: A+B підтверджує HOME лише при відпущеному стику та повністю зупиненій осі Z.
    if (confirmCombo && axisZ == 0.0f && !stepperZ->isRunning()) {
        completeStartupZHome();
    }
}
