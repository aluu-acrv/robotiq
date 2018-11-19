by Amelia Luu 15/11/18

1. clone kinetic branch of robotiq
https://github.com/ros-industrial/robotiq/tree/kinetic-devel

2. rosdep install robotiq_modbus_tcp
3. sudo apt-get install ros-kinetic-soem
4. sudo apt-get install ros-kinetic-socketcan-interface

Follow instructions under Dependencies:
https://github.com/Danfoa/invite-robotics/wiki/Intallation?fbclid=IwAR3D7D45PbQEPxJVZ-tftb8P639zgyGHlLQTm2AA37h2eVDcAQYIA15YpIA


create exception in uart devices 
https://answers.ros.org/question/219395/ros-node-does-not-recognize-haptic-device/?fbclid=IwAR1txTb4qyiGnJ6sPhrAUBOhYmGryTnvj1Dvpm0C1B6WMQ-l3ENbHoECkWY


5. connect force torque sensor thorugh usb to your device
6. ``lsusb -v``
7. find "Future Technology..." 
    - get "id vendor" & "id product"

8. create 10-mydevices.rules file and add a line similar to this: 
    ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", MODE="666", GROUP="plugdev"

9. check you're a editing member of plugdev ``groups USERNAME``
10. update rules ``sudo udevadm trigger``

11. connect ft sensor and it should publish to a topic!
    ``rosrun robotiq_ft_sensor rq_sensor``