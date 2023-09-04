#!/bin/bash


##########################################################
## Make startup shell script

startup_shell_file="/usr/sbin/velocity_listener_package_auto_launch"

# Check if the file exists; create it if it doesn't
if [ ! -f "$startup_shell_file" ]; then
  sudo touch "$startup_shell_file"
fi

# Define the content to be added to /usr/sbin/ros_package_auto_launch
startup_shell_content="#!/bin/bash
source ${WBR914_PROJECT_PATH}/src/install/setup.bash
source /etc/ros/env.sh
export ROS_HOME=\$(echo ~pi)/.ros
ros2 run wbr914_velocity_package wbr914_velocity_listener --wait
# Operation Canceled
exit 125
"

# Check if the content already exists in the file
if grep -qF "$startup_shell_content" "$startup_shell_file"; then
  echo "Content already exists in $startup_shell_file"
else
  # If the content does not exist, append it to the file
  echo "$startup_shell_content" >> "$startup_shell_file"
  echo "Content added to $startup_shell_file"
fi

# Make the script executable
sudo chmod +x /usr/sbin/ros_package_auto_launch

##########################################################

## Create the service that runs on startup.
## Notice! user-name used is "pi". change it if you use other user-name.
# Define the path to the service file
service_file="/etc/systemd/system/ros_package.service"

# Check if the service file already exists; if not, create it
if [ ! -f "$service_file" ]; then
    touch "$service_file"
fi


service_content="[Unit]
Requires=roscore.service
After=network-online.target

[Service]
Type=simple
User=pi
ExecStart=$startup_shell_file

[Install]
WantedBy=multi-user.target
"

# Check if the content already exists in the file
if grep -qF "$service_content" "$service_file"; then
  echo "Content already exists in $service_file"
else
  # If the content does not exist, append it to the file
  echo "$service_content" >> "$service_file"
  echo "Content added to $service_file"
fi

# Reload systemd to read the new service definition and enable it.
sudo systemctl daemon-reload
sudo systemctl enable ros_package.service


