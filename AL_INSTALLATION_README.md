## Installation instructions specific for setting up Robotiq's FT300 sensor on Ubuntu

written by ALuu, last updated 24 March 2021

----

> NOTE: this clone step should NOT be required because it should have been added as a submodule as per instructions in overall [`~designrobotics-linishing/README.md`](../../../README.md)

1. clone kinetic branch of robotiq
https://github.com/ros-industrial/robotiq/tree/kinetic-devel

2. rosdep install robotiq_modbus_tcp
3. sudo apt-get install ros-melodic-soem ros-melodic-socketcan-interface

Follow instructions under Dependencies:
https://github.com/Danfoa/invite-robotics/wiki/Intallation?fbclid=IwAR3D7D45PbQEPxJVZ-tftb8P639zgyGHlLQTm2AA37h2eVDcAQYIA15YpIA

create exception in uart devices
https://answers.ros.org/question/219395/ros-node-does-not-recognize-haptic-device/?fbclid=IwAR1txTb4qyiGnJ6sPhrAUBOhYmGryTnvj1Dvpm0C1B6WMQ-l3ENbHoECkWY

5. connect force torque sensor thorugh usb to your device
6. ``lsusb -v``
7. find "Future Technology..."
    - get "id vendor" & "id product"
probably looks something like this...

  idVendor           0x0403 Future Technology Devices International, Ltd
  idProduct          0x6015 Bridge(I2C/SPI/UART/FIFO)

8. create a new file in /etc/udev/rules.d named 10-mydevice.rules and add a line similar to this:

    sudo touch /etc/udev/rules.d/10-mydevice.rules
    sudo nano /etc/udev/rules.d/10-mydevice.rules

    copy paste the following if the id's are the same:
        ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", MODE="666", GROUP="plugdev"

9. check you're a editing member of plugdev ``groups``
    check that your USERNAME is part of the list
10. update rules ``sudo udevadm trigger``

11. connect ft sensor and it should publish to a topic!
    ``rosrun robotiq_ft_sensor rq_sensor``


RECOMMENDED: Decreasing the sensor latency

More information regarding why there is a latency to begin with: https://github.com/ros-industrial/robotiq/issues/124

In `./bashrc` script, under alias you can add these two lines:

```
alias fts100Hz="echo '2' | sudo tee --append /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
alias checkFTSFreq="cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
```

> You may need to double check which ttyUSBx the FT300 has been attached to. To find out, go to 
```
cd /sys/bus/usb-serial/devices
```

There should be a series of folders titled ttyUSB0, ttyUSB1 etc. Find out which folder has the file `latency_timer`. This is the USB number that hsould be in the alias commands. 


Then in a new terminal (with the FTS connected through USB) type `fts100Hz` and `checkFTSFreq`.

fts100Hz should result in '2'
and if that's done correctly, checkFTSFreq should now print '2' as well (instead of '16' which is when publishing at 60Hz)

The robotiq ft sensor should publish at 100Hz instead of 60Hz now. You can check by:

```
rostopic hz /robotiq_ft_wrench
```

> This must be repeated everytime you reconnect the fts sensor serially in that lifetime