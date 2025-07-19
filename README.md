# Rotator

**Rotator** is a spherical parallel manipulator (SPM) powered by four smart servos and controlled via a Python asyncio application. It allows real-time orientation control across an 80° hemispherical workspace and is compatible with `rotctl` protocol tools like [GPredict](https://gpredict.oz9aec.net/).

The project combines mechanical simplicity (a passive spherical joint) with flexible and modern software design, making it ideal for satellite tracking, directional sensors, and robotics experimentation.

![diagram](https://github.com/JimBlaney/rotator/blob/main/assets/animation.gif?raw=true)  

## 📐 Mechanical Overview

- **Arms**: 4 identical arms, each composed of two 60° arc-length segments (total 120° per arm), radius 106 mm
- **Base Joints**: Mounted at a –60° inclination from the horizontal
- **Top Joints**: Mounted at a +20° inclination from the horizontal
- **Center Joint**: 20 mm steel ball rigidly mounted to the base via a vertical column
- **Top Plate**: Rests in a shallow cup on the ball, providing passive rotation constraint
- **Workspace**: 80°

## ⚙️ Hardware Requirements

| Component              | Details                                                      |
|------------------------|--------------------------------------------------------------|
| Servos                 | 4x LX-16A (with ID set to 1, 2, 3, 4 for N, E, S, W arms respectively)|
| Microcontroller        | Raspberry Pi (I'm using a Zero 2 W)                      |
| Power Supply           | 5V @ 5A+ (dedicated power for servos)                        |
| Serial Comm            | BusLinker (e.g. `/dev/serial0`)           |

## 📁 Software Overview

This project uses Python 3.11+ and `asyncio` to manage real-time control of the manipulator. The servo class is inspired by [ethanlipson/PyLX-16A](https://github.com/ethanlipson/PyLX-16A).

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
* Convert video of real-life build to GIF and add to README.md
* Add dynamic engage/disengage to the servos (via the Platform class) during periods of no input

