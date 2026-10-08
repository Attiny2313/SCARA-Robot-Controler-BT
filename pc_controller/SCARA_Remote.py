#!/usr/bin/env python3
"""
SCARA Remote - Windows/Linux PC controller for the ESP32 SCARA firmware.

Protocol:
  PCHELLO
  PCPAD,LX,LY,RX,RY,BUTTONS,DPAD
  PCBYE

Axes are sent as integers from -1000 to 1000.
"""

import math
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque

import serial
from serial.tools import list_ports


BAUDRATE = 115200
SEND_PERIOD_MS = 50
HELLO_RETRY_S = 0.8

BTN_A = 1 << 0
BTN_B = 1 << 1
BTN_X = 1 << 2
BTN_START = 1 << 3
BTN_MODE = 1 << 4
BTN_TEACH = 1 << 5

DPAD_UP = 1 << 0
DPAD_DOWN = 1 << 1
DPAD_LEFT = 1 << 2
DPAD_RIGHT = 1 << 3


class VirtualJoystick(tk.Canvas):
    def __init__(self, master, title, size=190, **kwargs):
        super().__init__(
            master,
            width=size,
            height=size,
            highlightthickness=1,
            relief="sunken",
            **kwargs,
        )
        self.size = size
        self.center = size / 2
        self.radius = size * 0.36
        self.knob_radius = size * 0.09
        self.x_value = 0.0
        self.y_value = 0.0

        self.create_text(self.center, 14, text=title, font=("Segoe UI", 10, "bold"))
        self.create_oval(
            self.center - self.radius,
            self.center - self.radius,
            self.center + self.radius,
            self.center + self.radius,
            width=2,
        )
        self.create_line(
            self.center - self.radius,
            self.center,
            self.center + self.radius,
            self.center,
        )
        self.create_line(
            self.center,
            self.center - self.radius,
            self.center,
            self.center + self.radius,
        )
        self.knob = self.create_oval(0, 0, 0, 0, width=2)
        self.value_text = self.create_text(
            self.center, self.size - 13, text="X 0.00   Y 0.00"
        )
        self._draw_knob()

        self.bind("<Button-1>", self._move)
        self.bind("<B1-Motion>", self._move)
        self.bind("<ButtonRelease-1>", self._release)

    def _move(self, event):
        dx = event.x - self.center
        dy = event.y - self.center
        distance = math.hypot(dx, dy)
        if distance > self.radius and distance > 0:
            scale = self.radius / distance
            dx *= scale
            dy *= scale

        self.x_value = max(-1.0, min(1.0, dx / self.radius))
        self.y_value = max(-1.0, min(1.0, dy / self.radius))
        self._draw_knob()

    def _release(self, _event=None):
        self.x_value = 0.0
        self.y_value = 0.0
        self._draw_knob()

    def _draw_knob(self):
        x = self.center + self.x_value * self.radius
        y = self.center + self.y_value * self.radius
        r = self.knob_radius
        self.coords(self.knob, x - r, y - r, x + r, y + r)
        self.itemconfigure(
            self.value_text,
            text=f"X {self.x_value:+.2f}   Y {self.y_value:+.2f}",
        )

    def values(self):
        return self.x_value, self.y_value


class ScaraRemote(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SCARA Remote - USB")
        self.geometry("930x690")
        self.minsize(850, 620)

        self.ser = None
        self.handshake_ok = False
        self.last_hello = 0.0
        self.rx_buffer = ""
        self.buttons = 0
        self.dpad = 0
        self.keys_down = set()
        self.log_lines = deque(maxlen=120)

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Rozłączony")
        self.arm_var = tk.StringVar(value="SAFE")
        self.work_var = tk.StringVar(value="-")
        self.control_var = tk.StringVar(value="-")
        self.home_var = tk.StringVar(value="-")
        self.estop_var = tk.StringVar(value="-")
        self.relay_var = tk.StringVar(value="-")
        self.points_var = tk.StringVar(value="-")
        self.speed_var = tk.StringVar(value="-")

        self.a1_var = tk.StringVar(value="0.00°")
        self.a2_var = tk.StringVar(value="0.00°")
        self.z_var = tk.StringVar(value="0.00 mm")
        self.x_var = tk.StringVar(value="0.00 mm")
        self.y_var = tk.StringVar(value="0.00 mm")
        self.rot_var = tk.StringVar(value="0.0°")
        self.grip_var = tk.StringVar(value="0.0°")

        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(SEND_PERIOD_MS, self.tick)

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Port COM:").pack(side="left")
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=24, state="readonly"
        )
        self.port_combo.pack(side="left", padx=(6, 6))
        ttk.Button(top, text="Odśwież", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(top, text="Połącz", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=(6, 14))

        ttk.Label(top, text="Stan:").pack(side="left")
        ttk.Label(top, textvariable=self.connection_var, font=("Segoe UI", 10, "bold")).pack(
            side="left", padx=(5, 0)
        )

        status = ttk.LabelFrame(self, text="Robot", padding=8)
        status.pack(fill="x", padx=10, pady=(0, 8))

        status_items = [
            ("A1", self.a1_var),
            ("A2", self.a2_var),
            ("Z", self.z_var),
            ("TCP X", self.x_var),
            ("TCP Y", self.y_var),
            ("ROT", self.rot_var),
            ("GRIP", self.grip_var),
            ("WORK", self.work_var),
            ("CONTROL", self.control_var),
            ("PC", self.arm_var),
            ("HOME Z", self.home_var),
            ("E-STOP", self.estop_var),
            ("RELAY", self.relay_var),
            ("Punkty", self.points_var),
            ("AUTO", self.speed_var),
        ]

        for i, (name, var) in enumerate(status_items):
            frame = ttk.Frame(status)
            frame.grid(row=i // 8, column=i % 8, padx=8, pady=3, sticky="w")
            ttk.Label(frame, text=f"{name}:").pack(side="left")
            ttk.Label(frame, textvariable=var, font=("Segoe UI", 9, "bold")).pack(
                side="left", padx=(3, 0)
            )

        control = ttk.Frame(self, padding=(10, 0, 10, 0))
        control.pack(fill="both", expand=True)

        left_col = ttk.LabelFrame(control, text="Sterowanie", padding=10)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 6))

        joy_frame = ttk.Frame(left_col)
        joy_frame.pack()
        self.left_joy = VirtualJoystick(joy_frame, "LEFT: ARM1/ARM2 lub TCP X/Y")
        self.left_joy.pack(side="left", padx=8)
        self.right_joy = VirtualJoystick(joy_frame, "RIGHT: ROT / Z")
        self.right_joy.pack(side="left", padx=8)

        button_frame = ttk.Frame(left_col)
        button_frame.pack(fill="x", pady=(12, 4))

        self._hold_button(button_frame, "A  Grip +", BTN_A).grid(row=0, column=0, padx=4, pady=4)
        self._hold_button(button_frame, "B  Grip -", BTN_B).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(
            button_frame, text="X  Relay", command=lambda: self.pulse_button(BTN_X)
        ).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(
            button_frame, text="START  JOINT/TOOL", command=lambda: self.pulse_button(BTN_START)
        ).grid(row=0, column=3, padx=4, pady=4)

        ttk.Button(
            button_frame, text="MODE  MANUAL/AUTO", command=lambda: self.pulse_button(BTN_MODE)
        ).grid(row=1, column=0, padx=4, pady=4)

        ttk.Button(
            button_frame,
            text="TEACH  Zapisz punkt",
            command=self.send_teach_command,
        ).grid(row=1, column=1, padx=4, pady=4, sticky="ew")

        ttk.Button(
            button_frame,
            text="KASUJ PROGRAM",
            command=self.confirm_clear_program,
        ).grid(row=1, column=2, padx=4, pady=4, sticky="ew")

        ttk.Button(
            button_frame,
            text="HOME Z  A+B",
            command=lambda: self.pulse_button(BTN_A | BTN_B, 320),
        ).grid(row=1, column=3, padx=4, pady=4)

        dpad_frame = ttk.LabelFrame(left_col, text="AUTO speed", padding=6)
        dpad_frame.pack(pady=(7, 0))
        ttk.Button(
            dpad_frame, text="▲ +", width=8, command=lambda: self.pulse_dpad(DPAD_UP)
        ).grid(row=0, column=1, padx=3, pady=2)
        ttk.Button(
            dpad_frame, text="◀ 100%", width=8, command=lambda: self.pulse_dpad(DPAD_LEFT)
        ).grid(row=1, column=0, padx=3, pady=2)
        ttk.Button(
            dpad_frame, text="▼ -", width=8, command=lambda: self.pulse_dpad(DPAD_DOWN)
        ).grid(row=1, column=1, padx=3, pady=2)

        help_text = (
            "Klawiatura: WASD = lewy joystick, IJKL = prawy joystick, "
            "Q/E = A/B, R = relay, SPACE = START, M = MODE, T = zapisz punkt, "
            "H = HOME Z (A+B), strzałki = D-pad AUTO.\n"
            "Po połączeniu pozostaw wszystko neutralnie przez chwilę, aż PC zmieni stan SAFE → ARMED."
        )
        ttk.Label(left_col, text=help_text, wraplength=560, justify="left").pack(
            fill="x", pady=(10, 0)
        )

        right_col = ttk.LabelFrame(control, text="Log", padding=6)
        right_col.pack(side="right", fill="both", expand=False, padx=(6, 0))
        self.log = tk.Text(right_col, width=38, height=28, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True)

        warning = ttk.Label(
            self,
            text=(
                "USB jest dodatkowym źródłem sterowania. Fizyczny E-STOP pozostaje nadrzędny. "
                "Brak pakietów z PC przez 600 ms zatrzymuje ruch."
            ),
            padding=(10, 7),
        )
        warning.pack(fill="x")

    def _hold_button(self, master, text, mask):
        btn = tk.Button(master, text=text)
        btn.bind("<ButtonPress-1>", lambda _e: self.set_button(mask, True))
        btn.bind("<ButtonRelease-1>", lambda _e: self.set_button(mask, False))
        return btn

    def _bind_keys(self):
        self.bind_all("<KeyPress>", self.key_press)
        self.bind_all("<KeyRelease>", self.key_release)

    def key_press(self, event):
        key = event.keysym.lower()
        was_down = key in self.keys_down
        self.keys_down.add(key)

        if key == "q":
            self.set_button(BTN_A, True)
        elif key == "e":
            self.set_button(BTN_B, True)
        elif key == "r":
            self.set_button(BTN_X, True)
        elif key == "space":
            self.set_button(BTN_START, True)
        elif key == "m":
            self.set_button(BTN_MODE, True)
        elif key == "t" and not was_down:
            self.send_teach_command()
        elif key == "h":
            self.set_button(BTN_A | BTN_B, True)
        elif key == "up":
            self.dpad |= DPAD_UP
        elif key == "down":
            self.dpad |= DPAD_DOWN
        elif key == "left":
            self.dpad |= DPAD_LEFT
        elif key == "right":
            self.dpad |= DPAD_RIGHT

    def key_release(self, event):
        key = event.keysym.lower()
        self.keys_down.discard(key)

        if key == "q":
            self.set_button(BTN_A, False)
        elif key == "e":
            self.set_button(BTN_B, False)
        elif key == "r":
            self.set_button(BTN_X, False)
        elif key == "space":
            self.set_button(BTN_START, False)
        elif key == "m":
            self.set_button(BTN_MODE, False)
        elif key == "t":
            pass
        elif key == "h":
            self.set_button(BTN_A | BTN_B, False)
        elif key == "up":
            self.dpad &= ~DPAD_UP
        elif key == "down":
            self.dpad &= ~DPAD_DOWN
        elif key == "left":
            self.dpad &= ~DPAD_LEFT
        elif key == "right":
            self.dpad &= ~DPAD_RIGHT

    def set_button(self, mask, pressed):
        if pressed:
            self.buttons |= mask
        else:
            self.buttons &= ~mask

    def pulse_button(self, mask, duration_ms=160):
        self.set_button(mask, True)
        self.after(duration_ms, lambda: self.set_button(mask, False))

    def pulse_dpad(self, mask, duration_ms=160):
        self.dpad |= mask
        self.after(duration_ms, lambda: setattr(self, "dpad", self.dpad & ~mask))

    def send_teach_command(self):
        if not self.ser or not self.ser.is_open or not self.handshake_ok:
            messagebox.showwarning("SCARA Remote", "Najpierw połącz aplikację z ESP32.")
            return
        self.write_line("PCTEACH")
        self.append_log("PC → ESP32: zapisz punkt TEACH")

    def confirm_clear_program(self):
        if not self.ser or not self.ser.is_open or not self.handshake_ok:
            messagebox.showwarning("SCARA Remote", "Najpierw połącz aplikację z ESP32.")
            return

        if not messagebox.askyesno(
            "SCARA Remote",
            "Skasować cały zapisany program AUTO?\n\nTej operacji nie można cofnąć.",
        ):
            return

        self.write_line("PCCLEAR")
        self.append_log("PC → ESP32: kasuj program AUTO")

    def refresh_ports(self):
        ports = list(list_ports.comports())
        values = [f"{p.device} — {p.description}" for p in ports]
        self.port_combo["values"] = values

        if values and self.port_var.get() not in values:
            preferred = next(
                (
                    value
                    for value in values
                    if any(
                        token in value.lower()
                        for token in ("cp210", "ch340", "usb serial", "uart", "esp32")
                    )
                ),
                values[0],
            )
            self.port_var.set(preferred)

    def selected_port(self):
        value = self.port_var.get().strip()
        if not value:
            return ""
        return value.split(" — ", 1)[0]

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.selected_port()
        if not port:
            messagebox.showwarning("SCARA Remote", "Wybierz port COM.")
            return

        try:
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = BAUDRATE
            ser.timeout = 0
            ser.write_timeout = 0.2
            ser.dtr = False
            ser.rts = False
            ser.open()
            self.ser = ser
        except Exception as exc:
            messagebox.showerror("SCARA Remote", f"Nie można otworzyć {port}:\n{exc}")
            return

        self.handshake_ok = False
        self.last_hello = 0.0
        self.connection_var.set(f"{port} - handshake...")
        self.connect_button.configure(text="Rozłącz")
        self.append_log(f"Połączono z {port}. Oczekiwanie na ESP32...")

    def disconnect(self):
        if self.ser:
            try:
                if self.ser.is_open:
                    if self.handshake_ok:
                        self.write_line("PCPAD,0,0,0,0,0,0")
                        self.write_line("PCBYE")
                    self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.handshake_ok = False
        self.buttons = 0
        self.dpad = 0
        self.connection_var.set("Rozłączony")
        self.connect_button.configure(text="Połącz")
        self.arm_var.set("SAFE")
        self.append_log("Rozłączono.")

    def keyboard_axes(self):
        lx = (-1.0 if "a" in self.keys_down else 0.0) + (
            1.0 if "d" in self.keys_down else 0.0
        )
        ly = (-1.0 if "w" in self.keys_down else 0.0) + (
            1.0 if "s" in self.keys_down else 0.0
        )
        rx = (-1.0 if "j" in self.keys_down else 0.0) + (
            1.0 if "l" in self.keys_down else 0.0
        )
        ry = (-1.0 if "i" in self.keys_down else 0.0) + (
            1.0 if "k" in self.keys_down else 0.0
        )
        return lx, ly, rx, ry

    def current_axes(self):
        mlx, mly = self.left_joy.values()
        mrx, mry = self.right_joy.values()
        klx, kly, krx, kry = self.keyboard_axes()

        lx = klx if klx != 0.0 else mlx
        ly = kly if kly != 0.0 else mly
        rx = krx if krx != 0.0 else mrx
        ry = kry if kry != 0.0 else mry

        return tuple(max(-1.0, min(1.0, value)) for value in (lx, ly, rx, ry))

    def write_line(self, line):
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except Exception as exc:
            self.append_log(f"Błąd portu: {exc}")
            self.disconnect()

    def send_pad(self):
        lx, ly, rx, ry = self.current_axes()
        values = [int(round(v * 1000.0)) for v in (lx, ly, rx, ry)]
        self.write_line(
            f"PCPAD,{values[0]},{values[1]},{values[2]},{values[3]},"
            f"{self.buttons},{self.dpad}"
        )

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return
            data = self.ser.read(waiting).decode("utf-8", errors="replace")
        except Exception as exc:
            self.append_log(f"Błąd odczytu: {exc}")
            self.disconnect()
            return

        self.rx_buffer += data
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip("\r").strip()
            if line:
                self.handle_line(line)

    def handle_line(self, line):
        if line == "PCACK,HELLO,1":
            self.handshake_ok = True
            port = self.selected_port()
            self.connection_var.set(f"{port} - połączony")
            self.append_log("ESP32: handshake OK")
            return

        if line == "PCACK,NEED_HELLO,0":
            self.handshake_ok = False
            self.append_log("ESP32 wymaga ponownego handshake.")
            return

        if line.startswith("PCACK,TEACH,"):
            result = line.split(",", 2)[2]
            if result.startswith("OK"):
                self.append_log(f"TEACH OK: {result}")
            else:
                self.append_log(f"TEACH odrzucony: {result}")
            return

        if line.startswith("PCACK,CLEAR,"):
            result = line.split(",", 2)[2]
            if result == "OK":
                self.append_log("Program AUTO skasowany.")
            else:
                self.append_log(f"Kasowanie odrzucone: {result}")
            return

        if line.startswith("PCSTATE,"):
            self.update_status(line)
            return

        self.append_log(line)

    def update_status(self, line):
        fields = {}
        for item in line.split(",")[1:]:
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key] = value

        try:
            self.a1_var.set(f"{float(fields.get('A1', 0)):.2f}°")
            self.a2_var.set(f"{float(fields.get('A2', 0)):.2f}°")
            self.z_var.set(f"{float(fields.get('Z', 0)):.2f} mm")
            self.x_var.set(f"{float(fields.get('X', 0)):.2f} mm")
            self.y_var.set(f"{float(fields.get('Y', 0)):.2f} mm")
            self.rot_var.set(f"{float(fields.get('ROT', 0)):.1f}°")
            self.grip_var.set(f"{float(fields.get('GRIP', 0)):.1f}°")
        except ValueError:
            pass

        self.work_var.set(fields.get("WORK", "-"))
        self.control_var.set(fields.get("CONTROL", "-"))
        self.arm_var.set("ARMED" if fields.get("ARMED") == "1" else "SAFE")
        self.home_var.set("OK" if fields.get("HOMEZ") == "1" else "WYMAGANY")
        self.estop_var.set("AKTYWNY" if fields.get("ESTOP") == "1" else "OK")
        self.relay_var.set("ON" if fields.get("RELAY") == "1" else "OFF")
        self.points_var.set(fields.get("POINTS", "-"))
        speed = fields.get("SPEED", "-")
        self.speed_var.set(f"{speed}%")

    def append_log(self, text):
        timestamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"[{timestamp}] {text}")
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.insert("end", "\n".join(self.log_lines))
        self.log.see("end")
        self.log.configure(state="disabled")

    def tick(self):
        if self.ser and self.ser.is_open:
            self.read_serial()
            now = time.monotonic()

            if not self.handshake_ok:
                if now - self.last_hello >= HELLO_RETRY_S:
                    self.write_line("PCHELLO")
                    self.last_hello = now
            else:
                self.send_pad()

        self.after(SEND_PERIOD_MS, self.tick)

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    ScaraRemote().mainloop()
