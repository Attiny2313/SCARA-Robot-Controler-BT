# SCARA Remote (PC / USB)

A small desktop controller for the SCARA firmware. It communicates with the ESP32
through the same USB serial port used by PlatformIO / Serial Monitor.

## Requirements

- Python 3.10+ recommended
- Tkinter (included with the standard Windows Python installer)
- pyserial

Install the dependency:

```powershell
cd pc_controller
py -m pip install -r requirements.txt
```

Run directly:

```powershell
py SCARA_Remote.py
```

On Windows you can also double-click `run_windows.bat`. It checks for `pyserial`,
installs it from `requirements.txt` when needed, and starts the controller.

Close PlatformIO Serial Monitor before connecting the application because only one
program can normally own the COM port at a time.

## Connection and safety

1. Connect the ESP32 over USB.
2. Start `SCARA_Remote.py`.
3. Select the ESP32 COM port and press **Połącz**.
4. The application performs the `PCHELLO` handshake automatically.
5. Leave both virtual joysticks and all buttons neutral for at least 300 ms.
6. The status changes from **SAFE** to **ARMED**.
7. After every ESP32 restart, establish Z HOME before normal MANUAL operation.

The firmware expects a fresh `PCPAD` packet continuously. If packets stop for more
than 600 ms, manual motion is stopped. If AUTO is running, it is paused.

The physical E-STOP remains authoritative.

## Controls

- Left virtual joystick: ARM1/ARM2 in JOINT, TCP X/Y in TOOL.
- Right virtual joystick: rotation on X and Z on Y.
- **A / B**: gripper.
- **X**: relay.
- **START**: JOINT / TOOL in MANUAL; start / pause / resume in AUTO.
- **MODE**: MANUAL / AUTO.
- **TEACH / Zapisz punkt**: sends a dedicated save command to the ESP32.
- **KASUJ PROGRAM**: asks for confirmation and sends a dedicated clear command.
- The physical PCB TEACH button still keeps its original behavior: short press saves, hold at least 1.8 s clears.
- **HOME Z A+B**: confirms startup Z HOME when Z is stopped.
- AUTO D-pad: up/down changes speed, left restores 100%.

Keyboard shortcuts:

- `W A S D` - left joystick
- `I J K L` - right joystick
- `Q / E` - A / B
- `R` - relay
- `Space` - START
- `M` - MODE
- `T` - TEACH
- `H` - A+B for startup Z HOME
- Arrow keys - AUTO D-pad

## Serial protocol

The desktop app sends:

```text
PCHELLO
PCPAD,LX,LY,RX,RY,BUTTONS,DPAD
PCTEACH
PCCLEAR
PCBYE
```

Axes are integers from `-1000` to `1000`.

Button mask:

- bit 0 = A
- bit 1 = B
- bit 2 = X
- bit 3 = START
- bit 4 = MODE
- bit 5 = TEACH

D-pad mask:

- bit 0 = UP
- bit 1 = DOWN
- bit 2 = LEFT
- bit 3 = RIGHT

The firmware returns machine-readable status lines beginning with `PCSTATE,`.
