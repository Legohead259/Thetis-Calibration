#! /bin/bash

# Update the internal packages and upgrade to latest versions
sudo apt update && apt upgrade -y

# Install and configure git
sudo apt install git -y
git config --global user.name Legohead259
git config --global user.email legohead259@gmail.com

# Install ROS2 (https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt install ros-iron-desktop

# Setup ROS2 to be CLI accessible
source /opt/ros/iron/setup.bash

# Ensure all future terminals can access the `ros2` CLI
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Get PIP
sudo apt install python3-pip -y

# Install RPi.GPIO (https://askubuntu.com/questions/1230947/gpio-for-raspberry-pi-gpio-group)
pip3 install rpi.gpio
sudo apt install rpi.gpio-common    # Fixes "No access to /dev/mem" error with Python RPi.GPIO
sudo adduser "${USER}" dialout      # Fixes "No access to /dev/mem" error with Python RPi.GPIO

# Install PyQT
pip3 install PyQT6

# Restart system
sudo reboot

# VS Code Extensions
# 1. Python
# 2. Pylance
# 3. AutoDocstring
# 4. CMake Tools