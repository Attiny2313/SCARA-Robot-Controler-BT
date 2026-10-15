# Experimental PyBot-style HOME for ARM1 and ARM2

This experimental mode was added on the `dev` branch. It requires **no limit switches**, but also has **no contact or stall detection** using A4988. Successful completion only means the expected step commands were executed; it does **not** prove accurate homing.

## Source parameters
PyBot Configuration.h: ARM1 operating [-120,+120] deg, ARM2 [-140,+140] deg. Its Robot.ino calibration uses hard-stop assumptions with absolute maxima 121 and 146 degrees, respectively. This port deliberately limits the added overrun to 6 degrees and jogs at 4 deg/s. It does not copy PyBot's 1000/1600 forced step bursts.

## Procedure
1. Confirm real mechanical stops are compatible with the PyBot absolute maxima, direction polarities, gear coupling, motor-current settings, and available clearance.
2. Put the mechanism near its nominal 0/0 starting position **before boot** (the controller cannot determine the true starting angle without feedback).
3. Keep the working area clear and be ready to press the **hardware E-STOP**. Open the serial monitor at 115200 baud and send `HOME ARMS` followed by a newline.
4. ARM1 drives toward -121°, applies bounded 6° extra travel, sets an assumed reference, and returns to 0°. ARM2 then drives toward +146°, applies bounded 6° extra travel, sets an assumed reference, and returns to 0°.
5. Complete the existing manually confirmed HOME Z operation with the gamepad (A+B). Abort with `HOME STOP` or hardware E-STOP; reset is required after abort.

**Do not use unattended.** A stepper hitting a mechanical stop may skip steps, vibrate, heat, or damage the belt drive/printed structure. Position estimation is open-loop, so tolerances and gear backlash can make the resulting zero inaccurate. The sequence was not compiled on hardware or mechanically validated, and must not be treated as production-safe.
