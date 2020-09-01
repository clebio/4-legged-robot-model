# 4-legged-robot-model

This code is the last version that runs on the raspberry pi,
 for the simulation code that runs on PC and implements a PyBullet simulation, check the old branch (V1_PCsimulation).

Working on python3

-numpy, csv , evdev , pyserial and simple-pid needed to run the robot.

(run: robot_main_RPI.py)
_______________________________________________________________________


You can also run its telemetry. 

-Pandas and matplotlib needed.

(run: run_telemetry.py)
_______________________________________________________________________

All the info at:   https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot



Here is a simple diagram of how the robot is working:
![alt text](https://github.com/miguelasd688/4-legged-robot-model/blob/master/esquema.jpg)

## Debugging Joystick 

```python
python -m evdev.evtest
udevadm info -a -n /dev/input/event0
udevadm info -q all /dev/bus/usb/001/007 # or whatever ID lsusb shows

usermod  -a -G input pi

# http://www.janosgyerik.com/adding-udev-rules-for-usb-debugging-android-devices/

root@robotica:~# lsusb  |  grep ogitech
Bus 001 Device 007: ID 046d:c21f Logitech, Inc. F710 Wireless Gamepad [XInput Mode]

root@robotica:~# cat /etc/udev/rules.d/99-logitech.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="046d", ATTR{idProduct}=="c21f", MODE="0660", GROUP="input", SYMLINK+="logitechF710"
```


## Arduino

* https://github.com/sudar/Arduino-Makefile/

```
sudo apt-get install arduino-mk arduino-core arduino -y
```

## Development loop


On local machine:
```sh
rsync -v --exclude 3D-printable-files/ -a ./ pi@robotica.local:quadruped
```

On the Pi:
```sh
pushd quadruped/arduino-code/arduino_com/; make upload; popd'
```

### Arduino CLI on Raspberry Pi

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

### Validate

```sh
$ arduino-cli board list
Port         Type              Board Name                FQBN             Core
/dev/ttyACM0 Serial Port (USB) Arduino Mega or Mega 2560 arduino:avr:mega arduino:avr
/dev/ttyAMA0 Serial Port       Unknown
```

### Libraries

    arduino-cli board list
    arduino-cli lib  search lsm9ds1
    arduino-cli lib  install "SparkFun LSM9DS1 IMU"

### Makefile

cat > Makefile <<EOF
ARDUINO_DIR  = /usr/share/arduino
BOARD_TAG    = mega2560
ARDUINO_PORT = /dev/ttyACM0
ARDUINO_LIBS = Serial "SparkFun LSM9DS1 IMU"

include /usr/share/arduino/Arduino.mk
EOF

### Usage

    make upload