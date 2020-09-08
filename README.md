# Quadruped robot

Quadruped robot hobby project. Fully open-source. A robot dog under the umbrella of the [Spot Micro][spotmicro] project. This fork based off of [Miguel Parrilla's amazing work][miguel].

Read the [detailed documentation](docs/README.md) or see below for general housekeeping.

[spotmicro]: https://spotmicroai.readthedocs.io/en/latest/
[miguel]: https://github.com/miguelasd688/4-legged-robot-model
[gait]: https://en.wikipedia.org/wiki/Horse_gait

## Original

This code is the last version that runs on the raspberry pi,
 for the simulation code that runs on PC and implements a PyBullet simulation, check the old branch (V1_PCsimulation).

Working on python3

* numpy, csv , evdev , pyserial and simple-pid needed to run the robot.

Run:

    robot_main_RPI.py

You can also run its telemetry. 

* Pandas and matplotlib needed.

Run:

    run_telemetry.py

All the info at [Hackaday project][hackaday]

[hackaday]: https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot


Here is a simple diagram of how the robot is working:
![alt text](https://github.com/miguelasd688/4-legged-robot-model/blob/master/esquema.jpg)


# New Material

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

### Arduino CLI on Raspberry Pi

    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

### Validate

```sh
$ arduino-cli board list
Port         Type              Board Name                FQBN             Core
/dev/ttyACM0 Serial Port (USB) Arduino Mega or Mega 2560 arduino:avr:mega arduino:avr
/dev/ttyAMA0 Serial Port       Unknown
```

On the Pi:
```sh
pi@robotica:~/quadruped/arduino-code/test_serial $ python read_serial.py

You sent me: Hello from Raspberry Pi!
You sent me: Hello from Raspberry Pi!
Found control markers: >>>SERIAL<<< 1599019635.065907 72 0.612566295533153
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

## Development loop

On local machine:
```sh
rsync -v --exclude 3D-printable-files/ -a ./ pi@robotica.local:quadruped

fswatch -e '.*.tmp' -e '.git/.*' -i .*py$ -i .*ino$ -r . | xargs -I{} \
    rsync -v --exclude 3D-printable-files/ --exclude .git/ -a ./ pi@robotica.local:quadruped

```

On the Pi:
```sh
cd quadruped
pushd arduino; make upload; popd
python servo_calibration.py
```
