# Quadruped Robot

* Hardware
    * 3D printing
    * Bill of Materials
    * Workbench and tools
    * Gamepad controller
* Electronics
    * 18650s
    * soldering
    * load and power distribution
* Software
    * IMU
    * Servos, Arduino and Serial
    * Input
    * event loop
* Controls
    * Kinematics
    * Movement, gait planning and transitions
* Housekeeping
    * Project planning
    * Tests and build system
    * Contributing
    * LICENSE
* References


## Gamepad Controller

On the Pi:
```sh
sudo apt-get install libusb-dev
apt-get install blueman
sudo bluetoothctl <<EOF
agent on
default-agent
discoverable on
EOF
```

[bt_cite]: https://wouterdeschuyter.be/blog/configure-a-ps3-controller-to-automatically-connect-to-a-raspberry-pi

## A word about planning

A project of this size is complicated. There are multiple systems, each of which will take time to build and debug: hardware, electrical, software, controls. Plan the project and your time accordingly. When working with hardware and software-controlled electronics, inevitably something will break and need to be replaced. Have extra components on-hand or allow for time to acquire replacement bits.

[spotmicro]: https://spotmicroai.readthedocs.io/en/latest/
[miguel]: https://github.com/miguelasd688/4-legged-robot-model
[gait]: https://en.wikipedia.org/wiki/Horse_gait
