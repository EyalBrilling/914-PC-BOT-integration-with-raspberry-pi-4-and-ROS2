#!/bin/bash

WBR914_PROJECT_PATH="/home/pi/WhiteBox-PC-BOT-rpi4"
echo "Notice! WBR914_PROJECT_PATH const inside 'setting_node_on_startup,sh' is $WBR914_PROJECT_PATH. Change it inside file if needed"


##########################################################
## Make startup shell script

startup_shell_file="/usr/sbin/wbr914_node_auto_launch"

# Check if the file exists; remove its previous if it does
if [ -f "$startup_shell_file" ]; then
  rm $startup_shell_file
fi
sudo touch "$startup_shell_file"

# Define the content to be added to /usr/sbin/ros_package_auto_launch
## Notice! user-name used is "pi". change it if you use other user-name.
startup_shell_content="#!/bin/bash
source $WBR914_PROJECT_PATH/src/install/setup.bash
source /etc/ros/env.sh
export ROS_HOME=\$(echo ~pi)/.ros
ros2 run wbr914_package wbr914_node --wait
# Operation Canceled
exit 125
"

  # append content to the file
echo "$startup_shell_content" >> "$startup_shell_file"
echo "Content added to $startup_shell_file"


# Make the script executable
sudo chmod +x $startup_shell_file

##########################################################

## Create the service that runs on startup.
## Notice! user-name used is "pi". change it if you use other user-name.
# Define the path to the service file
service_file="/etc/systemd/system/ros_package.service"

# Check if the file exists; remove its previous if it does
if [ -f "$service_file" ]; then
  rm $service_file
fi
sudo touch "$service_file"


service_content="[Unit]
After=network-online.target

[Service]
Type=simple
User=pi
ExecStart=$startup_shell_file

[Install]
WantedBy=multi-user.target
"

# Add content to service file.
echo "$service_content" >> "$service_file"
echo "Content added to $service_file"

# Reload systemd to read the new service definition and enable it.
sudo systemctl daemon-reload
sudo systemctl enable ros_package.service


