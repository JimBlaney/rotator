# Rotator

**Rotator** is a spherical parallel manipulator (SPM) powered by four smart servos and controlled via a Python `asyncio` application. It allows real-time orientation control across an 80° hemispherical workspace and is compatible with `rotctl` protocol tools like [GPredict](https://gpredict.oz9aec.net/).

The project combines mechanical simplicity (a passive spherical joint) with flexible and modern software design, making it ideal for satellite tracking, directional sensors, and robotics experimentation.

![diagram](https://github.com/JimBlaney/rotator/blob/main/assets/animation.gif?raw=true)  

## 📐 Mechanical Overview

- **Arms**: 4 identical arms, each composed of two 60° arc-length segments (total 120° per arm), radius 106 mm
- **Base Joints**: Mounted at a –60° inclination from the horizontal
- **Top Joints**: Mounted at a +20° inclination from the horizontal
- **Center Joint**: 20 mm steel ball rigidly mounted to the base via a vertical column
- **Top Plate**: Rests in a shallow cup on the ball, providing passive rotation constraint
- **Workspace**: 80° (±40º pitch/roll)

## 🛒 Bill of Materials
| Qty | Component | Price (ea) | Link | Notes |
|----:|-----------|-----------:|------|-------|
| 4 | LX-16A Servo | $18.00 | [Amazon](https://www.amazon.com/dp/B073XY5NT1) | BusLinker board (required) |
| 1 | Raspberry Pi Zero 2 W | $17.99 | [MicroCenter](https://www.microcenter.com/product/683270/raspberry-pi-raspberry-pi-zero-w-2-with-headers) | with headers |
| 1 | MicroSD Card | $7.60 | [Amazon](https://www.amazon.com/SanDisk-Ultra-SDSQUNS-016G-GN3MN-UHS-I-microSDHC/dp/B074B4P7KD) | 16GB |
| 1 | Logic Level Shifter | $6.25 | [Amazon](https://www.amazon.com/dp/B0CL2R6K26) | |
| 1 | 12V Power Supply | $20.99 | [Amazon](https://www.amazon.com/dp/B07MXXXBV8) | |
| 1 | Buck Converter | $1.90 | [Amazon](https://www.amazon.com/dp/B096RC71DC) | |
| 1 | M6x20mm Ball Knob | $3.82 | [Amazon](https://www.amazon.com/dp/B0C7MZW9YY) | |
| 1 | M6x12mm Grub Screw | $0.13 | [Amazon](https://www.amazon.com/dp/B0DS9XWCCP) | |
| 4 | M3x15mm Screw | $0.01 | [Amazon](https://www.amazon.com/dp/B0B51BFSWZ) | |
| 8 | M2x15mm Screw | $0.01 | [Amazon](https://www.amazon.com/dp/B0D2KWKX5S) | |
| 4 | T25 Metal Servo Spline Insert | $3.50 | [Amazon](https://www.amazon.com/dp/B00371I4D4) | |
| 1 | 1/4"-20x1/2" Screw | $0.32 | [Amazon](https://www.amazon.com/dp/B07PMH6NXZ) | |
| 1 | 1/4"-20 Tripod Nut | $4.00 | [Amazon](https://www.amazon.com/dp/B07CFW4WM1) | |
| 8 | 10mm x 5mm x 4mm Bearing | $0.62 | [Amazon](https://www.amazon.com/dp/B082PRVZ7B) | outer diameter x inner diameter x depth|
| **Total**| | $154.08 | | 

## 🖨️ Printable 3D Models
| Qty | File | Description |
|---|----|----|
| 1 | top-5.1.obj | top plate & passive joint |
| 1 | stabilizer-bottom.obj | passive joint support |
| 1 | bottom-5.1.obj | base & servo mount
| 4 | arm-lower.obj | lower arm segment |
| 4 | arm-upper.obj | upper arm segment |

I used the Elegoo Centauri Carbon with PLA-CF on default settings.

## 📁 Software Overview

This project uses Python 3.11+ and `asyncio` to manage real-time control of the manipulator. The LX-16A servo class is inspired by [ethanlipson/PyLX-16A](https://github.com/ethanlipson/PyLX-16A).

### Project Structure

rotator/  
├── \_\_main\_\_.py # Main entry point (launches platform + rotctl server)  
├── platform.py # Core orientation logic + servo angle coordination  
├── lx16a.py # Servo abstraction for LX-16A serial protocol  
├── math_utils.py # Inverse kinematics and mechanical transforms  
├── rotctl.py # An `asyncio.Protocol` implementation of a minimal `rotctl` protocol  
├── requirements.txt # pip requirements   
└── README.md  

## 🔧 Installation

### 1. System Setup

- Flash Raspberry Pi OS
- Enable serial/UART
- Connect your servos via BusLinker or USB-TTL
- Clone this repo:

```bash
git clone https://github.com/JimBlaney/rotator.git
cd src
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m rotator -h # show the usage
```

## 🚀 Running the Rotator

Launch the control system:

```bash
cd src
source .venv/bin/activate
python -m rotator rotctl # binds to 0.0.0:4533 by default
```

This will:

* Initialize servos
* Start an asynchronous loop for servo position updates
* Start a TCP listener on port 4533 for remote control commands

## 🎮 Remote Control (rotctl / GPredict)

### GPredict Setup

Open: Edit → Preferences → Interfaces → Rotators

Add a new rotator:

* Host: IP address of your Raspberry Pi
* Port: 4533
* Minimum Elevation: 50°

Assign to a satellite and begin tracking

### Manual Control

You can also send raw commands using netcat, rotctl, or any TCP tool:

```bash
echo "P 180 50\n" | nc <pi_ip> 4533
```

This points the rotator to azimuth 180°, elevation 50°.

## 🧠 Internal Architecture

### Platform:
- Accepts target azimuth/elevation or pitch/roll
- Solves inverse kinematics for servo angles
- Interpolates motion

### Servo:
- Communicates over LX-16A serial protocol
- Can be tested or used standalone

### Rotctl Server:
- Parses P AZ EL commands
- Updates platform target orientation
- Compatible with hamlib and Gpredict

## ⚙️ Calibration

Running the calibration routine will set the angle offset to the servo EEPROM

```bash
cd src
source .venv/bin/activate
python -m rotator calibrate # follow instructions
```

## 🛡 License

This project is licensed under the MIT License.

## 🚧 TODO

* Add more detail to the README.md
* Add dynamic engage/disengage to the servos (via the Platform class) during periods of no input

