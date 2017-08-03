# PyByrobotPetrone
Python Implementation for [Byrobot Petrone](http://en.byrobot.co.kr/eng/petrone/) Controller

![Petrone Drone](http://byrobot.co.kr/eng/wp-content/uploads/petrone_A-1024x576.jpg)

+ Support Byrobot's Petrone Drone
+ Publishes on-board sensors in ROS Message formats
+ Subscribes control command in ROS Message formats
+ Supports for multiple petrones

----

## ROS

### Parameters

The ros package supports the following arguments:

+ petrone_addr : Bluetooth Address(eg. 50:8c:b1:5f:16:b7)

+ petrone_hci : if petrone_addr is not provided, this package scan all petrone devices and automatically connect one of them.

  + if you want to scan, 'sudo' permission is required.
  + sudo -E roslaunch ....

### Subscribers

#### cmd_vel

+ linear.y: roll
+ linear.x: pitch
+ angular.z: yawrate
+ linear.z: thrust

All of them is in the range of [-100 to 100]

#### cmd_fly

Send a command to change flight mode.
 
+ Ready = 1
+ TakeOff = 2
+ Flight = 3
+ Flip = 4
+ Stop = 5
+ Landing = 6
+ Reverse = 7
+ Accident = 8
+ Error = 9

### Publishers

#### IMU

+ TODO

#### Battery

+ Float32
+ 0 to 100 percentage

### Demo

#### TeleOperation Demo by Joystick

Check [/launch/control_joy.launch](/launch/control_joy.launch).
 
+ Take off : Y Button
+ Landing : A Button

## Dependencies

### bluepy & BlueZ for Bluetooth LE Connection

+ bluepy : https://github.com/IanHarvey/bluepy 
+ BlueZ >= 5.29 Required

#### Installing BlueZ from sources

The `bluetoothd` daemon provides BlueZ's D-Bus interfaces that is accessed by the GATT SDK to communicate with Bluetooth devices. The following commands download BlueZ 5.44 sources, built them and replace any pre-installed `bluetoothd` daemon. It's not suggested to remove any pre-installed BlueZ package as its deinstallation might remove necessary Bluetooth drivers as well.

1. `sudo systemctl stop bluetooth`
2. `sudo apt-get update`
3. `sudo apt-get install libusb-dev libdbus-1-dev libglib2.0-dev libudev-dev libical-dev libreadline-dev libdbus-glib-1-dev unzip`
4. `cd`
5. `mkdir bluez`
6. `cd bluez`
7. `wget http://www.kernel.org/pub/linux/bluetooth/bluez-5.44.tar.xz`
8. `tar xf bluez-5.44.tar.xz`
9. `cd bluez-5.44`
10. `./configure --prefix=/usr --sysconfdir=/etc --localstatedir=/var --enable-library`
11. `make`
12. `sudo make install`
13. `sudo ln -svf /usr/libexec/bluetooth/bluetoothd /usr/sbin/`
14. `sudo install -v -dm755 /etc/bluetooth`
15. `sudo install -v -m644 src/main.conf /etc/bluetooth/main.conf`
16. `sudo systemctl daemon-reload`
17. `sudo systemctl start bluetooth`
18. `bluetoothd --version` # should now print 5.44

## References

[1] https://dronefighters.github.io/Documents/kr/products/petrone/protocol/

[2] https://github.com/IanHarvey/bluepy

[3] https://ianharvey.github.io/bluepy-doc/

[4] https://github.com/myungsup1250/Petrone_BLE_Control_With_Console/blob/791d2aa507a7816ace573df3cecc08aed9ee650c/Petrone%20BLE%20Console%20Sample%20C%2B%2B/petrone.cpp#L58

