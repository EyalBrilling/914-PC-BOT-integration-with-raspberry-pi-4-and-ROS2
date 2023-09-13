#!/bin/bash

##########################################################
## This script will make you an automatic connection file to be put on the pi4.

read -p "Enter wifi name(SSID): " wifiName
echo "You entered $wifiName"
read -p "Enter wifi password(blank if no password): " password
echo "You entered $password"

auto_connection_file="./${wifiName}_automatic_connection.yaml"

# Define the content to be added to /usr/sbin/ros_package_auto_launch
auto_connection_content="network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        $wifiName:
          password: $password
"

  # append content to the file
echo "$auto_connection_content" >> "$auto_connection_file"
echo "Content added to $auto_connection_file"


