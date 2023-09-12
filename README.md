# WhiteBox914 Robot,raspberry pi 4 and ROS2 integration

This project includes drivers,ROS2 nodes and instructions for using the WhiteBox914 in combination with raspberry pi 4 and ROS2.

Using this project will enable listening to robot commands via the pi4 on it's startup,running ros nodes to communicate with the robot and write new packages for it.

This README gives an overview on starting using the project.  
Other README in the project:

- [docs/raspberry_pi_setup.md](docs/raspberry_pi_setup.md): Setup the raspberry pi 4 for listening to commands and transmit them to the robot on startup.
- [src/README.md](src/README.md): Instructions for downloading ROS2,building packages and running them.

## Table of Contents

- [WhiteBox914 Robot,raspberry pi 4 and ROS2 integration](#whitebox914-robotraspberry-pi-4-and-ros2-integration)
  - [Table of Contents](#table-of-contents)
  - [Setup the project](#setup-the-project)
    - [Setup the pi4](#setup-the-pi4)
    - [Setup the workspace](#setup-the-workspace)
      - [Add the project path as an ENV variable](#add-the-project-path-as-an-env-variable)
      - [Downloading ROS2 and build packages](#downloading-ros2-and-build-packages)
  - [Running a ROS2 node](#running-a-ros2-node)
  - [Additional Resources](#additional-resources)
    - [Technical information about the robot](#technical-information-about-the-robot)
    - [Original robot drivers](#original-robot-drivers)
    - [WhiteBox-PC-BOT dropbox](#whitebox-pc-bot-dropbox)
    - [Player](#player)

## Setup the project

Setup of the project is divided into two parts:

1) Setup the pi4 itself on the robot.
2) Setup the workspace.

The setup work on the pi4 is intended to be a one time thing.
Once all setup is done,the pi4 will wait for commands the moment it gets power.

### Setup the pi4

Follow the instructions under [docs/raspberry_pi_setup.md](docs/raspberry_pi_setup.md)

### Setup the workspace

This is the computer where you will program + run ROS nodes to send robot commands.

#### Add the project path as an ENV variable

Add the following line to your .bashrc file:

```bash
export WBR914_PROJECT_PATH="/path/to/your/project"
```

#### Downloading ROS2 and build packages

All ROS packages of the project are under `src` folder.  
Read the [src/README.md](src/README.md) for instructions.

## Running a ROS2 node

Everytime a new terminal is opened, the built packages environment variables need to be sourced.

```shell
source ${WBR914_PROJECT_PATH}/src/install/setup.bash
```

This line can also be added to your .bashrc file for convenience.

After building packages,Running ROS executables is done by running the command:

```shell
ros2 run <package_name> <executable_name>
```

For example,to run wbr914 publisher in the `wbr914_velocity_package` package run:

```shell
ros2 run wbr914_velocity_package wbr914_velocity_publisher_continuous_basic
```

This command wil run a publisher that sends continuous velocity commands to the velocity topic by user request.

Assuming you already setup the pi4 - the pi4 will wait for commands on the velocity topic. Once you send a command,the robot will move.

## Additional Resources

### Technical information about the robot

Information about the robot sensors,power sources and more can be found under:

[docs/tech_specifications/](docs/tech_specifications/)

### Original robot drivers

The previous drivers written by the company can be found under the [player_driver](/player_driver) folder.  
In the previous iteration of the robot,player platform was used.

Notice this driver uses player structs and functions. It won't work with ROS.

### WhiteBox-PC-BOT dropbox

Dropbox with manuals from the original company.
https://www.dropbox.com/home/WhiteBox-PC-Bot

### Player

wbr914 Isn't supposed to use player anymore, but if for some reason you want to to test it with player,you can download it from [player github](https://github.com/playerproject/player).

And a manual with explanation of using player with the robot is found under [docs/robot_using_player/WBR_914_PC-BOT_old_Linux-Version.pdf](docs/robot_using_player/WBR_914_PC-BOT_old_Linux-Version.pdf)
