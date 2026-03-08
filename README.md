# Haptic control
A Linux-based project aimed at teleoperation using Haptic 3DSystem via UDP. 

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
- nlohamnn_json 3.11.3

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