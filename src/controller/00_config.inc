/*
 * SCARA Robot Controller / Sterownik robota SCARA
 *
 * PL: Sterownik 3-osiowego robota SCARA z ręcznym sterowaniem z pada,
 *     zapisem punktów TEACH i automatycznym odtwarzaniem programu.
 * EN: Controller for a 3-axis SCARA robot with gamepad manual control,
 *     TEACH point storage and automatic program playback.
 *
 * PL: Homing jest wykonywany ręcznie. Przed uruchomieniem operator musi
 *     ustawić robota w pozycji HOME zdefiniowanej poniżej.
 * EN: Homing is manual. Before startup, the operator must place the robot
 *     in the HOME position defined below.
 */

#include <Arduino.h>
#include <Bluepad32.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// PINY / PIN ASSIGNMENTS
// ============================================================
constexpr uint8_t PIN_LED_BT       = 4;
constexpr uint8_t PIN_LED_ERROR    = 16;
constexpr uint8_t PIN_LED_STATUS   = 17;

// PL: Objaw na obecnej płytce wskazuje na sterowanie LED BT stanem niskim.
//     Jeśli LED zachowuje się odwrotnie, zmień BT_LED_ACTIVE_LOW na false.
// EN: The current board symptom suggests an active-low BT LED.
constexpr bool BT_LED_ACTIVE_LOW = true;
constexpr uint32_t BT_LED_BLINK_MS = 500;

constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_ADDRESS_PRIMARY = 0x3C;
constexpr uint8_t OLED_ADDRESS_SECONDARY = 0x3D;

constexpr uint8_t PIN_STEP_ARM1    = 13;
constexpr uint8_t PIN_DIR_ARM1     = 26;
constexpr uint8_t PIN_STEP_ARM2    = 23;
constexpr uint8_t PIN_DIR_ARM2     = 14;
constexpr uint8_t PIN_STEP_Z       = 18;
constexpr uint8_t PIN_DIR_Z        = 19;

constexpr uint8_t PIN_SERVO_ROTATE = 27;
constexpr uint8_t PIN_SERVO_GRIP   = 32;

constexpr uint8_t PIN_BUTTON_TEACH = 25;
constexpr uint8_t PIN_BUTTON_MODE  = 33;
constexpr uint8_t PIN_ESTOP        = 39;

// PL: Wejścia krańcówek są zachowane sprzętowo na przyszłość.
//     Aktualna wersja programu używa ręcznego homingu.
// EN: Endstop inputs are kept in hardware for future use.
//     The current firmware uses manual homing.
constexpr uint8_t PIN_ENDSTOP_Z    = 34;
constexpr uint8_t PIN_ENDSTOP_ARM1 = 35;
constexpr uint8_t PIN_ENDSTOP_ARM2 = 36;

// ============================================================
// KONFIGURACJA MECHANIKI / MECHANICAL CONFIGURATION
// ============================================================
constexpr float ARM_LENGTH_1_MM = 91.61f;
constexpr float ARM_LENGTH_2_MM = 97.528f;

constexpr float MOTOR_STEPS_REV = 200.0f;
constexpr float MICROSTEPS      = 16.0f;

constexpr float GEAR_ARM1 = 3.875f;
constexpr float GEAR_ARM2 = 7.280f;
constexpr float Z_SCREW_LEAD_MM = 2.0f;
constexpr float GEAR_Z          = 1.0f;

// PL: Znak osi pozwala odwrócić kierunek logiczny bez przepinania przewodów.
// EN: Axis sign allows logical direction reversal without rewiring the motor.
constexpr int8_t SIGN_ARM1 = 1;
constexpr int8_t SIGN_ARM2 = 1;
constexpr int8_t SIGN_Z    = 1;

constexpr float STEPS_PER_DEG_ARM1 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM1 / 360.0f;
constexpr float STEPS_PER_DEG_ARM2 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM2 / 360.0f;
constexpr float STEPS_PER_MM_Z =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_Z / Z_SCREW_LEAD_MM;

// ============================================================
// RĘCZNY HOME / MANUAL HOME
// ============================================================
// PL: Przed włączeniem lub resetem robot musi zostać ręcznie ustawiony
//     w pozycji HOME. Program przyjmuje te wartości jako pozycję aktualną.
// EN: Before power-up or reset the robot must be manually placed in HOME.
//     The firmware assumes these values are the actual startup position.
constexpr float HOME_ARM1_DEG = 115.0f;
constexpr float HOME_ARM2_DEG = -147.0f;
constexpr float HOME_Z_MM     = 0.0f;
constexpr float HOME_TOOL_ROTATE_DEG = 90.0f;
constexpr float HOME_GRIP_DEG        = 30.0f;

// ============================================================
// OGRANICZENIA PROGRAMOWE / SOFTWARE LIMITS
// ============================================================
// PL: Ograniczenia chronią mechanikę w trybach MANUAL, TOOL i AUTO.
// EN: These limits protect the mechanism in MANUAL, TOOL and AUTO modes.
constexpr float ARM1_MIN_DEG = -115.0f;
constexpr float ARM1_MAX_DEG =  115.0f;
constexpr float ARM2_MIN_DEG = -147.0f;
constexpr float ARM2_MAX_DEG =  147.0f;
constexpr float Z_MIN_MM = 0.0f;
constexpr float Z_MAX_MM = 300.0f;
constexpr float TOOL_ROTATE_MIN_DEG = 0.0f;
constexpr float TOOL_ROTATE_MAX_DEG = 180.0f;
constexpr float GRIP_MIN_DEG = 15.0f;
constexpr float GRIP_MAX_DEG = 95.0f;

// ============================================================
// PRĘDKOŚCI I PRZYSPIESZENIA / SPEEDS AND ACCELERATIONS
// ============================================================
// PL: ARM1 i ARM2 mają tę samą prędkość kątową w sterowaniu ręcznym.
// EN: ARM1 and ARM2 use the same angular speed in manual control.
constexpr float ARM1_JOG_SPEED_DEG_S = 25.0f;
constexpr float ARM2_JOG_SPEED_DEG_S = 25.0f;

// PL: Prędkość osi Z po prawidłowym ustawieniu MS1/MS2/MS3.
// EN: Z-axis speed with MS1/MS2/MS3 configured correctly.
constexpr float Z_JOG_SPEED_MM_S     = 1.5f;
constexpr float TOOL_XY_SPEED_MM_S   = 20.0f;

// PL: Bazowe prędkości programu automatycznego.
// EN: Base speeds used by the automatic program.
constexpr float AUTO_ARM1_SPEED_DEG_S = 20.0f;
constexpr float AUTO_ARM2_SPEED_DEG_S = 20.0f;
constexpr float AUTO_Z_SPEED_MM_S     = 1.5f;

// PL: Override prędkości AUTO dotyczy ramion. D-pad góra/dół zmienia
//     wartość o 10%, a D-pad lewo przywraca 100%. Oś Z pozostaje bez zmian.
// EN: AUTO speed override affects the arms. D-pad up/down changes it
//     by 10%, while D-pad left restores 100%. Z speed remains unchanged.
constexpr uint16_t AUTO_ARM_SPEED_PERCENT_DEFAULT = 100;
constexpr uint16_t AUTO_ARM_SPEED_PERCENT_MIN     = 50;
constexpr uint16_t AUTO_ARM_SPEED_PERCENT_MAX     = 250;
constexpr uint16_t AUTO_ARM_SPEED_PERCENT_STEP    = 10;

constexpr float TOOL_ROTATE_SPEED_DEG_S = 35.0f;
constexpr float GRIP_SPEED_DEG_S        = 45.0f;

constexpr int SERVO_ROTATE_MIN_US = 500;
constexpr int SERVO_ROTATE_MAX_US = 2500;
constexpr int SERVO_GRIP_MIN_US   = 500;
constexpr int SERVO_GRIP_MAX_US   = 2500;
constexpr float SERVO_FILTER_ALPHA = 0.20f;

// PL: Twarde limity częstotliwości STEP są dodatkowym zabezpieczeniem,
//     niezależnym od prędkości zadawanej przez JOINT, TOOL lub AUTO.
// EN: Hard STEP frequency limits provide an additional safety layer,
//     independent of speeds requested by JOINT, TOOL or AUTO control.
constexpr uint32_t ARM1_MAX_STEP_HZ = 10000;
constexpr uint32_t ARM2_MAX_STEP_HZ = 10000;

// PL: Przy 1600 STEP/mm: 2400 Hz = 1.5 mm/s.
// EN: At 1600 STEP/mm: 2400 Hz = 1.5 mm/s.
constexpr uint32_t Z_MAX_STEP_HZ    = 2400;

constexpr uint32_t ARM1_ACCEL = 12000;
constexpr uint32_t ARM2_ACCEL = 5000;

// PL: Przy 1600 STEP/mm: 3000 STEP/s² = 1.875 mm/s².
// EN: At 1600 STEP/mm: 3000 STEP/s² = 1.875 mm/s².
constexpr uint32_t Z_ACCEL    = 3000;

// ============================================================
// CZASY I AUTO / TIMING AND AUTO SETTINGS
// ============================================================
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t SERVO_PERIOD_MS = 20;
constexpr uint32_t DISPLAY_PERIOD_MS = 250;
constexpr uint32_t GAMEPAD_TIMEOUT_MS = 500;
constexpr uint32_t GAMEPAD_ARM_NEUTRAL_MS = 300;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t TEACH_LONG_PRESS_MS = 1800;
constexpr uint32_t DEFAULT_POINT_DWELL_MS = 400;
constexpr int16_t JOYSTICK_DEADZONE = 90;
constexpr float MAX_CONTROL_DT_S = 0.05f;
constexpr uint8_t MAX_PROGRAM_POINTS = 20;
constexpr bool AUTO_LOOP_PROGRAM = true;

// ============================================================
// STRUKTURY I STANY / DATA TYPES AND STATES
// ============================================================
enum class ControlMode : uint8_t {
    JOINT,
    TOOL
};

enum class OperatingMode : uint8_t {
    MANUAL,
    AUTO
};

enum class AutoState : uint8_t {
    IDLE,
    MOVE,
    DWELL,
    PAUSED,
    FINISHED,
    FAULT
};

struct JointPosition {
    float arm1Deg;
    float arm2Deg;
    float zMm;
};

struct CartesianPosition {
    float xMm;
    float yMm;
    float zMm;
};

// PL: Jeden punkt programu zawiera pozycję osi, pozycje serw i czas postoju.
// EN: One program point stores joint positions, servo targets and dwell time.
struct ProgramPoint {
    JointPosition joints;
    float toolRotateDeg;
    float gripperDeg;
    uint32_t dwellMs;
};

struct RobotState {
    JointPosition joints;
    JointPosition actualJoints;
    CartesianPosition tcp;
    CartesianPosition actualTcp;

    float toolRotateDeg;
    float gripperDeg;

    ControlMode controlMode;
    OperatingMode operatingMode;
    AutoState autoState;

    bool estopLatched;
    bool controllerConnected;
    bool gamepadArmed;
};

// ============================================================
// OBIEKTY I STAN GLOBALNY / GLOBAL OBJECTS AND STATE
// ============================================================
FastAccelStepperEngine stepperEngine;
FastAccelStepper* stepperArm1 = nullptr;
FastAccelStepper* stepperArm2 = nullptr;
FastAccelStepper* stepperZ    = nullptr;

Servo servoRotate;
Servo servoGrip;
Preferences preferences;
ControllerPtr controllers[BP32_MAX_GAMEPADS];
RobotState robot;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

ProgramPoint programPoints[MAX_PROGRAM_POINTS];
uint8_t programPointCount = 0;
uint8_t currentProgramPoint = 0;
bool autoMotionStarted = false;
uint32_t autoDwellStartedMs = 0;

uint32_t lastControlMs = 0;
uint32_t lastServoUpdateMs = 0;
uint32_t lastGamepadPacketMs = 0;
uint32_t gamepadNeutralSinceMs = 0;
uint32_t lastModeButtonEdgeMs = 0;
uint32_t lastTeachButtonEdgeMs = 0;
uint32_t teachPressStartedMs = 0;
uint32_t lastBtLedBlinkMs = 0;
uint32_t lastDisplayMs = 0;

float servoRotateFilteredDeg = HOME_TOOL_ROTATE_DEG;
float servoGripFilteredDeg = HOME_GRIP_DEG;

uint16_t autoArmSpeedPercent = AUTO_ARM_SPEED_PERCENT_DEFAULT;
uint8_t previousAutoDpad = 0;

bool btLedBlinkState = false;
bool displayAvailable = false;
uint8_t displayI2cAddress = 0;

bool previousModeButton = LOW;
bool previousTeachButton = LOW;
bool previousPadStart = false;

