// SCARA Robot Controller / Sterownik robota SCARA
//
// Implementacja jest podzielona na sekcje .inc, ale wszystkie są dołączane
// do jednej jednostki kompilacji. Kolejność include ma znaczenie.

#include "controller/00_config.inc"
#include "controller/01_motion.inc"
#include "controller/02_auto.inc"
#include "controller/03_panel_bt.inc"
#include "controller/04_oled.inc"
#include "controller/05_manual_stepper.inc"
#include "controller/06_setup_loop.inc"
