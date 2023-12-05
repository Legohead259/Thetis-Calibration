#! /bin/bash

# Update system and install required libraries and applications (ROS2)
wget -O - https://raw.githubusercontent.com/Legohead259/Scripting/main/ROS2/ws-initialize-linux.sh | bash

# Install RPi.GPIO (https://askubuntu.com/questions/1230947/gpio-for-raspberry-pi-gpio-group)
pip3 install rpi.gpio
sudo apt install rpi.gpio-common -y # Fixes "No access to /dev/mem" error with Python RPi.GPIO
sudo adduser "${USER}" dialout      # Fixes "No access to /dev/mem" error with Python RPi.GPIO

# Install CircuitPython support
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-mlx90393

# Install PyQT
pip3 install PyQT5

# Install xIMU3 support
pip3 install ximu3

# Restart system
sudo reboot
