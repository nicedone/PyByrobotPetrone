# PyByrobotPetrone
Python Implementation for Byrobot Petrone Controller

## Setup



## References

[1] https://dronefighters.github.io/Documents/kr/products/petrone/protocol/

[2] https://github.com/IanHarvey/bluepy

[3] https://ianharvey.github.io/bluepy-doc/

[4] https://github.com/myungsup1250/Petrone_BLE_Control_With_Console/blob/791d2aa507a7816ace573df3cecc08aed9ee650c/Petrone%20BLE%20Console%20Sample%20C%2B%2B/petrone.cpp#L58


### Installing BlueZ from sources

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