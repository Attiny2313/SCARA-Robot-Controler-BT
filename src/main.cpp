// ============================================================
// PL: STEROWNIK ROBOTA SCARA
// EN: SCARA ROBOT CONTROLLER
// UA: КОНТРОЛЕР РОБОТА SCARA
// ============================================================
//
// PL: Implementacja jest podzielona na sekcje .inc, ale wszystkie są dołączane
//     do jednej jednostki kompilacji. Kolejność include ma znaczenie.
// EN: The implementation is split into .inc sections, but all of them are included
//     in a single compilation unit. Include order matters.
// UA: Реалізацію поділено на секції .inc, але всі вони підключаються
//     до однієї одиниці компіляції. Порядок include має значення.

#include "controller/00_config.inc"
#include "controller/01_motion.inc"
#include "controller/02_auto.inc"
#include "controller/03_panel_bt.inc"
#include "controller/03_startup_z_home.inc"
#include "controller/04_oled.inc"
#include "controller/05_manual_stepper.inc"
#include "controller/06_setup_loop.inc"
