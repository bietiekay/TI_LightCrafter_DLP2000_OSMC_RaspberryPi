#!/bin/bash

sudo apt update
sudo apt upgrade
sudo apt install i2ctools python-ip
sudo pip install setuptools

cd /home/osmc/DLP/dlp_lightcrafter-1.0.19/

sudo python setup.py install

sudo python LEDSet.py 300 300 300
