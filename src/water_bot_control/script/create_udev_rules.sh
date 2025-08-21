#!/bin/bash

echo "remap serial device to baseSerial"
echo "copying base_serial.rules to /etc/udev/rules.d/...."
source /usr/share/colcon_cd/function/colcon_cd.sh
colcon_cd water_bot_control
sudo cp script/base_serial.rules    /etc/udev/rules.d
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"