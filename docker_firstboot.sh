#! /bin/bash

# Setup ROS2 to be CLI accessible
source /opt/ros/iron/setup.bash

# Update the internal packages and upgrade to latest versions
apt update && apt upgrade -y

# Get PIP
apt install python3-pip -y

# Install RPi.GPIO
pip3 install rpi.gpio

# Configure git parameters
git config --global user.name Legohead259
git config --global user.email legohead259@gmail.com

# VS Code Extensions
# 1. Python
# 2. Pylance
# 3. AutoDocstring
# 4. CMake Tools

# Install CircuitPython support
pip3 install adafruit-blinka 
pip3 install adafruit-circuitpython-mlx90393 
# pip3 install adafruit-circuitpython-ssd1327 # CircuitPython for SSD1327 currently not working
pip3 install adafruit-circuitpython-seesaw


# Install luma.oled for SSD1327
apt install python3-dev python3-pip python3-numpy libfreetype6-dev libjpeg-dev build-essential
apt install libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libportmidi-dev
cd /home/
git clone https://github.com/rm-hull/luma.examples.git
cd luma.examples/
sudo -H pip install -e .
cd /home/