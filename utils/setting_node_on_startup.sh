#!/bin/bash

sudo mkdir /etc/ros

## Set URI for ROS_MASTER for ROS Core/launch
# Define the content to be added
ros_master_uri='export ROS_MASTER_URI=http://localhost:11311'
env_file='/etc/ros/env.sh'

# Check if the file exists; create it if it doesn't
if [ ! -f "$env_file" ]; then
  sudo touch "$env_file"
fi

# Check if the content already exists in the file
if grep -qF "$ros_master_uri" "$env_file"; then
  echo "Content already exists in $env_file"
else
  # If the content does not exist, append it to the file
  echo "$ros_master_uri" >> "$env_file"
  echo "Content added to $env_file"
fi

## Create the roscore service that runs on startup.
## Notice! user-name used is "pi". change it if you use other user-name.

#!/bin/bash

# Define the path to the service file
service_file="/etc/systemd/system/roscore.service"

# Check if the service file already exists; if not, create it
if [ ! -f "$service_file" ]; then
    touch "$service_file"
fi

# Define the content to be added to the service file
service_content="[Unit]
Description=ROScore service
After=network-online.target

[Service]
Type=forking
User=pi
ExecStart=/bin/sh -c \". /opt/ros/iron/setup.sh; . /etc/ros/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done\"

[Install]
WantedBy=multi-user.target
"

# Add the content to the service file
echo "$service_content" > "$service_file"
echo "ROScore service file created at $service_file"

# Reload systemd to read the new service definition and enable it.
sudo systemctl daemon-reload
sudo systemctl enable roscore.service

