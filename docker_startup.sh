#! /bin/bash

# Update the internal packages and upgrade to latest versions
apt update && apt upgrade -y

# Get PIP
apt install python3-pip -y

# Install RPi.GPIO
pip3 install rpi.gpio

# Configure git parameters
git config --global user.name Legohead259
git config --global user.email legohead259@gmail.com
