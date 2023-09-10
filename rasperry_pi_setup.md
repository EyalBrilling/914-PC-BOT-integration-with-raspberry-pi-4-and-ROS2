We will now set the rasperry pi 4 to run a node that listens for twist commands(velocity information) and send them to the robot. this node will get commands from the nodes we wrote to send those twist commands.

## Setup Operating system

### Do`nloading an operating system on the pi4.

Connect Your pi4 sd card to a USB stick and connect it to your computer.

using the rasperry pi4 imager:

```
sudo apt install rpi-imager
```

You have 2 ways to set up the pi 4 for running the node:
1) Using a UI(desktop) by downloading **Ubuntu Desktop 22.04.03 LTS**
2) Using a server by downloading **Ubuntu Server 22.04.03 LTS**

#### Ubuntu Server

https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview

Setup all of the advanced options(network,ssh,keyboard layout):

// picture

Note about network - assuming ssh is used,this network and cable network will be the only way you can connect to the pi4. If more networks are needed to be added,See `Setup automatic network connection with ssh` section in this file for details.

To connect using ssh,use the following format:

```
ssh <user_name>@<host_name>
```

user_name and host_name(ending with .local) were both setup in the advances options in the download. 
Notice,in most guides the ip port is used for conencting instand of host_name,but i found it to not work(Connection refused is returned by pi4)



#### Ubuntu Desktop

download **Ubuntu Desktop 22.04.03 LTS** r ** on the rasperry pi 4:

//Picture

**Note** Not Raspberry Pi OS, ROS2 is less supported on it.**

### Connect needed cables

2) Connect to the pi4 all neeeded cables - power(if using desktop - screen,keyboard and mouse)

### Setup internet

3) power the pi 4 on. Connect in the pi4 to the internet you will use. On this internet all commands will be communicated between nodes. your pi4 and computer you send command from must be connected on the same internet. 

## Setup workspace

### Clone the project to the pi4

we will use a listener(subscriber) ROS node to get velocity commands from other nodes. It is in the project.

 Clone the project using

```
git clone git@github.com:EyalBrilling/WhiteBox-PC-BOT-rpi4.git
```

### Add project path to .bashrc

our Cmake uses an ENV variable of the project path. Add it to your .bashrc:

echo 'export WBR914_PROJECT_PATH="<Path_to_project>"' >> ~/.bashrc && source ~/.bashrc

## Setup ROS2

The following 2 steps can be done for you by running 'rasperry_pi_setup.sh' in utils:

1) Give script premissions:

```shell
chmod +x rasperry_pi_setup.sh
```

2) run it

```shell
./rasperry_pi_setup.sh
```

### Download ROS2 on the pi4

5) Download ROS2 iron from:

https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

Follow the instructions. As we are using Ubuntu,Downloading ROS2 using Debian packages is done normally without the need for a docker.

### Setup USB premissions

Add your user to the dialout group:
```
sudo usermod -aG dialout <your_username>
```

## Get the velocity listener node executable

No` `e need to get the executable that runs the listener node.
To do so, `e need to build it.

### Source setup.bash

Needs to be done every terminal

```
source /opt/ros/iron/setup.bash
```

### Build the velocity subscriber

In the src folder of the project, run 

```
colcon build
```

The executable is under `src/build/wbr914_velocity_package/wbr914_velocity_listener`

## Testing subscriber works

Lets checkout if the procces until now went succefully. we will try to run the node:

First source the built workspace:

```
source install/setup.bash
```

Then try to run the code:

```
ros2 run wbr914_velocity_package wbr914_velocity_listener
```

## Setup the velocity subscriber node to run on pi4 startup.

As the pi4 will be put on the robot,we want it to run the node and listen for commands on turning on. 

You have a shell file under 'utils' called "setting_node_on_startup.sh" to run the commands. The shell script assumes linux user name 'pi' is used. Look at it for explanation on what it does.

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

- This is a ros1 guide. ROS2 doesnt use roscore as there is not master node in it anymore. So the roscore.service isn't needed.

**Test with:**

sudo systemctl start roscore.service
sudo systemctl status roscore.service

If you get errors in service,journalctl is helpful:

```
sudo journalctl -u ros_package.service
```

## Note on safety

All logic of `hen to stop and run are in the responsibolty of other nodes. Make sure your code is safe so that your robot `ont run a`ay and jump out the `indo`. In any code you `rite, on the deconstructor of your nodes publish a velocity of (0,0) command. This `ay, any lose of connection bet`een nodes `ill make the robot stop. 
