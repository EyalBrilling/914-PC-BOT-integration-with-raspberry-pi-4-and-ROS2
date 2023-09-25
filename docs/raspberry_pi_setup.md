# Raspberry pi 4 setup

We will now set the raspberry pi 4 to run a node on startup that:

- Creates a topic,listening for twist commands(velocity information) which in-turn send them to the robot.
- Creates 2 services that return position and velocity of the robot.

**another computer besides the pi4 is needed for the setup.**

## Setup Operating system

### Downloading an operating system on the pi4

Connect Your pi4 sd card to a USB stick and connect it to your computer.

using the raspberry pi4 imager:

```shell
sudo apt install rpi-imager
```

You have 2 ways to set up the pi 4 for running the node:

1) Using a server by downloading **Ubuntu Server 22.04.03 LTS**
2) Using a UI(desktop) by downloading **Ubuntu Desktop 22.04.03 LTS**

#### Ubuntu Server

Download Ubuntu Server to sim card with the help of the following guide:

https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview

![docs/photos/Server_pi4_install.png](/docs/photos/Server_pi4_install.png)

Setup all of the advanced options(network,ssh,keyboard layout):

![docs/photos/install_settings_1.png](/docs/photos/install_settings_1.png)
![docs/photos/install_settings_2.png](/docs/photos/install_settings_2.png)
![docs/photos/install_settings_3.png](/docs/photos/install_settings_3.png)

Note about network connection - assuming ssh is used,this default network and direct cable network will be the only way you can connect to the pi4 using ssh which means your only way to use it. If more networks are needed to be added.  See ['Setup automatic network connection with ssh'](#setup-automatic-network-connection-with-ssh) section in this file for details.

To connect using ssh,use the following format:

```shell
ssh <user_name>@<host_name>
```

user_name and host_name(ending with .local) were both setup in the advances options in the download.  
Notice,in most guides the ip port is used for conencting instand of host_name,but was found to not actually work (Connection refused is returned by pi4)

#### Ubuntu Desktop

download **Ubuntu Desktop 22.04.03 LTS** on the raspberry pi 4:

![docs/photos/Desktop_pi4_install.png](/docs/photos/Desktop_pi4_install.png)

**Note** Not Raspberry Pi OS, ROS2 is less supported on it.

### Connect needed cables

Connect to the pi4 all neeeded cables - power(if using desktop - screen,keyboard and mouse)

### Setup internet

Power the pi 4 on. Connect the pi4 to the internet you will use. On this internet all commands will be communicated between nodes. your pi4 and the computer you send commands from must be connected on the same internet.

## Setup pi4 project workspace

### Clone the project to the pi4

 Clone the project using

```shell
git clone git@github.com:EyalBrilling/WhiteBox-PC-BOT-rpi4.git
```

### Add project path to .bashrc

our Cmake uses an ENV variable of the project path. Add it to your .bashrc using the following command:

```shell
echo 'export WBR914_PROJECT_PATH="<Path_to_project>"' >> ~/.bashrc && source ~/.bashrc
```

## Setup ROS2

The following steps can be done for you by running 'raspberry_pi_setup.sh' in utils:

1) Give script premissions:

```shell
chmod +x raspberry_pi_setup.sh
```

2) Run it:

```shell
./raspberry_pi_setup.sh
```

### Download ROS2 on the pi4

Download ROS2 iron from:

https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

Follow the instructions. As we are using Ubuntu,Downloading ROS2 using Debian packages is done normally without the need for a docker.

### Setup USB premissions

Add your user to the dialout group:

```shell
sudo usermod -aG dialout <your_username>
```

### Source setup.bash

Needs to be done every terminal

```shell
source /opt/ros/iron/setup.bash
```

### Build the robot node

In the src folder of the project, run:

```shell
colcon build
```

The executable is under `src/build/wbr914_package/wbr914_node`

## Testing subscriber works

Lets checkout if the procces until now went succefully. we will try to run the node:

First source the built workspace:

```shell
source install/setup.bash
```

Then try to run the code:

```shell
ros2 run wbr914_package wbr914_node
```

## Setup the velocity subscriber service to run on pi4 startup

As the pi4 will be put on the robot,we want it to run the node and listen for commands on turning on.  
To do so,we will use a **service**. This service will run The ros2 node on startup of the pi4.

You have a shell file under `utils` called `setting_node_on_startup.sh`. This script runs commands that create the service.

 The shell script assumes linux user name 'pi' is used. Look at it for explanation on what it does.

1) Give it premissions:

    ```shell
    chmod +x setting_node_on_startup.sh 
    ```

2) Run it:

    ```shell
    sudo ./setting_node_on_startup.sh
    ```

The shell file is based on the following guide:

https://mshields.name/blog/2022-03-16-running-ros-nodes-on-boot/

**Notice changes from guide:**

- This is a ros1 guide. ROS2 doesnt use roscore as there is not master node in it anymore.

### Test service

The service will only active on the next reset of the pi4,so we start it manually.

```shell
sudo systemctl start ros_package.service
```

Use status to see the service status. If it's not active,something went wrong.

```shell
sudo systemctl status ros_package.service
```

If you get errors in service,journalctl is helpful:

```shell
sudo journalctl -u ros_package.service
```

If it works,that it! The pi4 is setup.

## Note on safety

All logic of when to stop and run are in the responsibility of other nodes - not the node that on the pi4.

Make sure your code is safe so that your robot wont run away and jump out the window.
The robot expects commands all the time,So once a node exits,assuming publishing of velocity commands stopped - the robot will also stop. So just make sure there are no uncontrollable nodes running.

## Setup automatic network connection with ssh

It might be the case we can't connect to the pi 4 using a cable and you want to change the default network you connect to in a place without other networks. As ssh conenction require a network, we can set the wifi to be connected to before hand.

1) In the utils folder,you have a shell script `automatic_network_connection_gen.sh`. You can use it on the pi4 itself to create the  conenction yaml. First,give it premissions:

    ```shell
    chmod +x ./automatic_network_connection_gen.sh
    ```

    Then run it:

    ```shell
    ./automatic_network_connection_gen.sh
    ```

    It will ask you for the wifi name and its password.
    What you will get is a file `<wifi_SSDI>_automatic_connection.yaml`:

    ```yaml
    network:
      version: 2
      renderer: networkd
      wifis:
        wlan0:
          dhcp4: true
          access-points:
            "Your_SSID_Name":
              password: "Your_Password"
    ```

    Where:  
    **Your_SSID_Name**: The SSID (name) of the Wi-Fi network.  
    **Your_Password**: The Wi-Fi network password.

2) Move the file from `utils` to `/etc/netplan/`. From utils run:

    ```shell
    sudo cp ./<wifi_SSDI>_automatic_connection.yaml /etc/netplan/
    ```

    Once a yaml of this format is in the netplan folder,the pi4 will try to connect into it on powerup.

## Common errors and solution

### colcon build CMakeError

```shell
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.
  .
  .
  .
```
  
  First,make sure you sourced ROS2 enviorment variables using:
  
```shell
source /opt/ros/iron/setup.bash
```
  
  If that still doesn't work - there is probably a problem with premission priviliages to the build folders because `sudo colcon build` was ran. solve it by:
  
1) Remove build folders in the src folder using

 ```shell
 sudo rm -rd src/install
 ```

Do the same for build and log folders.

2) build again `colcon build` without sudo

### open() Premission denied

Make sure user is part of the dialout group on linux by:

```shell
groups your_username
```

If not,add yourself to it:

```shell
sudo usermod -aG dialout <your_username>
```
