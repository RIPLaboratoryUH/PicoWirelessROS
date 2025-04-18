# PicoWirelessROS
This repository contains code that allows your Pico and host machine to communicate and setup a ROS node wirelessly. One script is meant to run on the Pico while the other runs on a host macine

The code is meant to work like this: Upon power up, the Pico W (or Pico 2 W) will boot up, connect to WiFi with the programmed SSID and password, and then transmit data to a specified IP address over UPD.

To make use of this, the user needs to run a script on a host PC to recieve the data and then create a ROS node that publishes it for every device on the network.

Users are meant to edit the "sensor_read()" function to output the desired data based on the sensor attached to the Pico. 
