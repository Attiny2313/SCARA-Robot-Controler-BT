# SCARA Robot Controller

Open-source controller project for the **PyBot SCARA robotic arm** and compatible SCARA robots, based on an **ESP32 D1 Mini** and a custom PCB.

The repository contains firmware, PCB documentation, technical documentation and 3D-printing resources used in the project.

## Build and upload

1. Install Visual Studio Code.
2. Install the PlatformIO extension.
3. Download or clone this repository.
4. Open the project folder in PlatformIO.
5. Connect the ESP32.
6. Build and upload the firmware.

## Credits

### PyBot / jjRobots

The mechanical construction of this robot is based on the **PyBot SCARA Robotic Arm** by **jjRobots**.

Many thanks to jjRobots for publishing the original mechanical design, printable parts and assembly concept that formed the mechanical basis of this project.

- https://www.jjrobots.com/scara-robotic-arm-by-jjrobots/
- https://cults3d.com/en/3d-model/gadget/scara-robotic-arm-open-source-with-control-app

### Alternative gripper

The alternative gripper is based on **Robot Gripper 9g Micro Servo** by **yisparyan**:

- https://www.thingiverse.com/thing:715525

Many thanks to yisparyan for sharing the design with the robotics community.

## License

Original content created for this repository is released under the **MIT License**, unless otherwise marked. This includes the original controller firmware, project documentation and original project files.

Third-party material keeps its original license:

- PyBot-derived mechanical geometry: **CC BY-NC-SA** as published by jjRobots.
- Alternative gripper derived from Thingiverse thing 715525 by yisparyan: **CC BY**.
- External software dependencies retain their own upstream licenses.

Because the PyBot mechanical design includes a **NonCommercial** restriction, those particular third-party 3D assets are not relicensed under MIT and should not be treated as commercially unrestricted open-source content.

See [`LICENSE`](LICENSE), [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) and [`3DPrinting/README.md`](3DPrinting/README.md) for details.
