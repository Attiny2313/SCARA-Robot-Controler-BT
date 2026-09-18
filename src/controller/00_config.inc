/*
 * SCARA Robot Controller / Sterownik robota SCARA
 *
 * PL: Sterownik 3-osiowego robota SCARA z ręcznym sterowaniem z pada,
 *     zapisem punktów TEACH i automatycznym odtwarzaniem programu.
 * EN: Controller for a 3-axis SCARA robot with gamepad manual control,
 *     TEACH point storage and automatic program playback.
 * UA: Контролер 3-осьового робота SCARA з ручним керуванням від геймпада,
 *     збереженням точок TEACH та автоматичним відтворенням програми.
 *
 * PL: Homing jest wykonywany ręcznie. Przed uruchomieniem operator musi
 *     ustawić robota w pozycji HOME zdefiniowanej poniżej.
 * EN: Homing is manual. Before startup, the operator must place the robot
 *     in the HOME position defined below.
 * UA: Повернення в HOME виконується вручну. Перед запуском оператор повинен
 *     встановити робота в положення HOME, визначене нижче.
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
// PL: PINY
// EN: PIN ASSIGNMENTS
// UA: ПРИЗНАЧЕННЯ ВИВОДІВ
// ============================================================
constexpr uint8_t PIN_LED_BT       = 4;
constexpr uint8_t PIN_LED_ERROR    = 16;
constexpr uint8_t PIN_LED_STATUS   = 17;

// PL: Dioda BT na obecnej płytce jest sterowana aktywnym stanem wysokim.
// EN: The BT LED on the current board is active-high.
// UA: Світлодіод BT на поточній платі активується високим рівнем.
constexpr bool BT_LED_ACTIVE_LOW = false;
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
// UA: Входи кінцевих вимикачів збережені апаратно для майбутнього використання.
//     Поточна версія програми використовує ручне встановлення HOME.
constexpr uint8_t PIN_ENDSTOP_Z    = 34;
constexpr uint8_t PIN_ENDSTOP_ARM1 = 35;
constexpr uint8_t PIN_ENDSTOP_ARM2 = 36;

// ============================================================
// PL: KONFIGURACJA MECHANIKI
// EN: MECHANICAL CONFIGURATION
// UA: КОНФІГУРАЦІЯ МЕХАНІКИ
// ============================================================
constexpr float ARM_LENGTH_1_MM = 91.61f;
constexpr float ARM_LENGTH_2_MM = 97.528f;

constexpr float MOTOR_STEPS_REV = 200.0f;
constexpr float MICROSTEPS      = 16.0f;

constexpr float GEAR_ARM1 = 4.5f;
constexpr float GEAR_ARM2 = 7.280f;
constexpr float Z_SCREW_LEAD_MM = 2.0f;
constexpr float GEAR_Z          = 1.0f;

// PL: Znak osi pozwala odwrócić kierunek logiczny bez przepinania przewodów.
// EN: Axis sign allows logical direction reversal without rewiring the motor.
// UA: Знак осі дозволяє змінити логічний напрямок без перепідключення двигуна.
constexpr int8_t SIGN_ARM1 = 1;
constexpr int8_t SIGN_ARM2 = 1;
constexpr int8_t SIGN_Z    = -1;

constexpr float STEPS_PER_DEG_ARM1 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM1 / 360.0f;
constexpr float STEPS_PER_DEG_ARM2 =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_ARM2 / 360.0f;
constexpr float STEPS_PER_MM_Z =
    MOTOR_STEPS_REV * MICROSTEPS * GEAR_Z / Z_SCREW_LEAD_MM;

// ============================================================
// PL: RĘCZNY HOME
// EN: MANUAL HOME
// UA: РУЧНЕ ВСТАНОВЛЕННЯ HOME
// ============================================================
// PL: Przed włączeniem lub resetem robot musi zostać ręcznie ustawiony
//     w pozycji HOME. Program przyjmuje te wartości jako pozycję aktualną.
// EN: Before power-up or reset the robot must be manually placed in HOME.
//     The firmware assumes these values are the actual startup position.
// UA: Перед увімкненням або скиданням робот потрібно вручну встановити
//     у положення HOME. Програма приймає ці значення як поточну стартову позицію.
constexpr float HOME_ARM1_DEG = 0.0f;
constexpr float HOME_ARM2_DEG = 0.0f;
constexpr float HOME_Z_MM     = 0.0f;
constexpr float HOME_TOOL_ROTATE_DEG = 90.0f;
constexpr float HOME_GRIP_DEG        = 90.0f;

// ============================================================
// PL: OGRANICZENIA PROGRAMOWE
// EN: SOFTWARE LIMITS
// UA: ПРОГРАМНІ ОБМЕЖЕННЯ
// ============================================================
// PL: Ograniczenia chronią mechanikę w trybach MANUAL, TOOL i AUTO.
// EN: These limits protect the mechanism in MANUAL, TOOL and AUTO modes.
// UA: Ці обмеження захищають механіку в режимах MANUAL, TOOL і AUTO.
constexpr float ARM1_MIN_DEG = -105.0f;
constexpr float ARM1_MAX_DEG =  105.0f;
constexpr float ARM2_MIN_DEG = -125.0f;
constexpr float ARM2_MAX_DEG =  125.0f;
constexpr float Z_MIN_MM = 0.0f;
constexpr float Z_MAX_MM = 300.0f;
constexpr float TOOL_ROTATE_MIN_DEG = 0.0f;
constexpr float TOOL_ROTATE_MAX_DEG = 180.0f;
constexpr float GRIP_MIN_DEG = 90.0f;
constexpr float GRIP_MAX_DEG = 180.0f;

// ============================================================
// PL: PRĘDKOŚCI I PRZYSPIESZENIA
// EN: SPEEDS AND ACCELERATIONS
// UA: ШВИДКОСТІ ТА ПРИСКОРЕННЯ
// ============================================================
// PL: ARM1 i ARM2 mają tę samą prędkość kątową w sterowaniu ręcznym.
// EN: ARM1 and ARM2 use the same angular speed in manual control.
// UA: ARM1 і ARM2 мають однакову кутову швидкість у ручному режимі.
constexpr float ARM1_JOG_SPEED_DEG_S = 25.0f;
constexpr float ARM2_JOG_SPEED_DEG_S = 25.0f;

// PL: Oś Z ma podniesioną prędkość bazową do 3 mm/s.
// EN: Z-axis base speed is raised to 3 mm/s.
// UA: Базову швидкість осі Z підвищено до 3 мм/с.
constexpr float Z_JOG_SPEED_MM_S     = 3.0f;
constexpr float TOOL_XY_SPEED_MM_S   = 20.0f;

// PL: Bazowe prędkości programu automatycznego.
// EN: Base speeds used by the automatic program.
// UA: Базові швидкості, що використовуються в автоматичній програмі.
constexpr float AUTO_ARM1_SPEED_DEG_S = 20.0f;
constexpr float AUTO_ARM2_SPEED_DEG_S = 20.0f;
constexpr float AUTO_Z_SPEED_MM_S     = 3.0f;

// PL: Override prędkości AUTO skaluje wszystkie trzy osie. D-pad góra/dół
//     zmienia wartość o 10%, a D-pad lewo przywraca 100%.
// EN: AUTO speed override scales all three axes. D-pad up/down changes it
//     by 10%, while D-pad left restores 100%.
// UA: Корекція швидкості AUTO масштабує всі три осі. D-pad вгору/вниз
//     змінює значення на 10%, а D-pad вліво повертає 100%.
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

// PL: Nie stosujemy dodatkowych twardych limitów częstotliwości STEP.
//     Częstotliwość wynika z zadanych prędkości i procentu AUTO.
// EN: No additional hard STEP-frequency caps are applied.
// UA: Додаткові жорсткі обмеження частоти STEP не застосовуються.
//     Частота визначається заданими швидкостями та відсотком AUTO.

constexpr uint32_t ARM1_ACCEL = 12000;
constexpr uint32_t ARM2_ACCEL = 5000;

// PL: Przy 1600 STEP/mm: 6000 STEP/s² = 3.75 mm/s².
// EN: At 1600 STEP/mm: 6000 STEP/s² = 3.75 mm/s².
// UA: За 1600 STEP/мм: 6000 STEP/с² = 3,75 мм/с².
constexpr uint32_t Z_ACCEL    = 6000;

// ============================================================
// PL: CZASY I AUTO
// EN: TIMING AND AUTO SETTINGS
// UA: ЧАСОВІ ПАРАМЕТРИ ТА НАЛАШТУВАННЯ AUTO
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
// PL: STRUKTURY I STANY
// EN: DATA TYPES AND STATES
// UA: СТРУКТУРИ ТА СТАНИ
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
// UA: Одна точка програми зберігає положення осей, цільові положення сервоприводів і час паузи.
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
// PL: OBIEKTY I STAN GLOBALNY
// EN: GLOBAL OBJECTS AND STATE
// UA: ГЛОБАЛЬНІ ОБ'ЄКТИ ТА СТАН
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