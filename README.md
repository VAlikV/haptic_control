# Haptic control
A Linux-based project aimed at teleoperation using Haptic 3DSystem via UDP. 


## Project structure

The software implements the haptic-device side of a bilateral teleoperation
system. It reads the operator's motion from a 3D Systems haptic device,
transforms it into Cartesian displacement and orientation commands, sends the
commands to the robot controller over UDP, and converts the returned interaction
forces into haptic feedback.

| File or directory | Description |
| --- | --- |
| `main.cpp` | Application entry point. It initializes the haptic device and the OpenHaptics scheduler, registers the real-time servo callback, reads the device state, applies the calculated feedback force, and performs an orderly shutdown. |
| `tele_params/tele_params.hpp` | Declares the haptic-state representation and the main teleoperation class, including coordinate transformations, scaling parameters, communication state, and Denavit--Hartenberg parameters. |
| `tele_params/tele_params.cpp` | Implements the teleoperation algorithm. It converts device motion into a 12-element command containing a 3-D displacement and a 3 x 3 rotation matrix. It also extracts force components from the received 25-element robot observation, transforms and scales them, and limits the resulting device force to +/-3 N. |
| `udp/udp_server.hpp` | Defines the templated, non-blocking UDP transport. JSON messages contain a sequence number and a numeric data array; duplicate and out-of-order packets are rejected. |
| `udp/udp_server.cpp` | Implements conversion between Eigen arrays and JSON arrays for UDP serialization and deserialization. |
| `udp/json.hpp` | Vendored single-header nlohmann JSON library used by the UDP transport. This is a third-party dependency. |
| `logger/logger.hpp`, `logger/logger.cpp` | Provide CSV-style logging of timestamps, commanded positions, measured positions, and button states for experiments. The corresponding calls in the current control loop are disabled by default. |
| `lockfree/lockfree.hpp` | Provides a single-producer, single-consumer lock-free ring buffer originally intended for asynchronous real-time logging. The current synchronous logger does not use it. |
| `for_haptics_DH.py` | Standalone Python utility for validating the haptic device's Denavit--Hartenberg parameters and forward-kinematics model and for plotting the resulting kinematic chain. |
| `visualize_logs.py` | Post-processing utility that resamples experimental logs, estimates velocity, calculates tracking errors, and plots commanded and measured motion. |
| `CMakeLists.txt` | Configures the C++20 build and links Eigen, OpenHaptics, POSIX real-time support, the mathematics library, and `ncurses`. |
| `include/` | Third-party OpenHaptics SDK headers; these files are not part of the original implementation presented in the article. |
| `lib/` | Precompiled OpenHaptics and graphics libraries required to build and run the application. These binaries are external dependencies. |
| `logs/` | Experimental measurements generated during teleoperation tests rather than executable source code. |

### Runtime data flow

During each servo cycle, `main.cpp` acquires the haptic-device state and passes
it to `TeleState`. The teleoperation module computes a 12-element motion command
and sends it through the UDP module. The remote controller returns a 25-element
observation vector. `TeleState` extracts the three force components, maps them
to the haptic-device coordinate frame, scales and limits them, and returns the
resulting vector to the servo callback for rendering on the device.

## Credentails
Made in Innopolis University
By:
- Alik Valiullin
- Dmitrii Mistrikov
- Ruslan Damindarov

## Dependencies
(Linux system)
- cmake
- Eigen 3.3

### Installation of dependencies (on ubuntu)
```bash
sudo apt install -y nlohmann-json3-dev libeigen3-dev cmake
```

## Installing Drivers and Examples for Haptic

### Download the installation scripts from the GitHub repository  
https://github.com/lexand59/3ds_touch_openhaptics

- `install-3ds-openhaptics-3.4.sh`
- `install-3ds-touch-drivers-2023.sh`

### Run the scripts

```bash
chmod +x <script_name>.sh
./<script_name>.sh
````

If necessary, install the required dependencies first (they will be listed in the terminal).
After installation, reboot the computer.

<!-- Copy the executables for Touch Setup and Touch diagnostics from the bin folder to /usr/bin -->

### Copy `LibPhantomIOLib42.so` to `/usr/lib`

### Create the `3DSystems` directory

```bash
sudo mkdir /usr/share/3DSystems
```

### Add the `GTDD_HOME` environment variable to `/etc/environment`

```bash
GTDD_HOME="/usr/share/3DSystems"
```

Then reboot the system and verify:

```bash
echo $GTDD_HOME
```

### Create the `config` directory

```bash
sudo mkdir /usr/share/3DSystems/config
sudo chmod 777 /usr/share/3DSystems/config
```

### Connect the Haptic device via USB

### Configure device access

```bash
sudo chmod 777 /dev/ttyACM0
```

### Add the device

```bash
sudo Touch_HeadlessSetup
```

## How to use
### Build
```bash
mkdir build
cd build
cmake ..
cmake --build .
```
### Run

```bash
./HapticControl
```

## Hardware
- Haptic Device
- Comuter with USB port