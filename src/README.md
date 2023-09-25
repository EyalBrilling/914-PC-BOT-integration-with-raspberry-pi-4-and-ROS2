# ROS2 packages/nodes README.md

Every folder in the src folder is a ROS2 package.  
The src folder structure:

- **wbr914_package** : Contains:
  - The node that runs on the robot itself via the pi4.
  - Under srv, service declrations.
- **node_examples** : Example nodes that communicate with the robot node, showing how to send commands to it.

## Download ROS2

https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

## Build wbr914 ROS2 packages

### Build packages

wbr914 ROS packages are under src.

Everytime a new terminal is opened, ROS2 environment variables need to be sourced:

```shell
source /opt/ros/iron/setup.bash
```

This line can also be added to your .bashrc file for convenience.

**To build the ROS packages run the following command from the src folder**

```shell
colcon build
```

### Build packages explanation

https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

### Example adding files to build in ROS

https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

## Run a package executable

Everytime a new terminal is opened, the built packages environment variables need to be sourced.

```shell
source ${WBR914_PROJECT_PATH}/src/install/setup.bash
```

This line can also be added to your .bashrc file for convenience.

After building packages,Running ROS executables is done by running the command:

```shell
ros2 run <package_name> <executable_name>
```

For example,to run wbr914 publisher in the `wbr914_package` package run:

```shell
ros2 run wbr914_package wbr914_velocity_publisher_continuous_basic
```

This command wil run the publisher that sends velocity commands to the robot.

## Build ROS2 packages from Visual studio code

We need to make a new vsc task that will run `colcon build`:

1) If you dont have one,create a new tasks.json file in the .vscode folder.
Can also open vsc command pallete(ctrl+shift+p) and search  "Tasks: Configure Default Build Task" To make the file.

2) Create the colcon build task. Here is a template(Make sure to change user_name):

```json
{
	"version": "2.0.0",
	"tasks": [
		{
			"type": "colcon",
			"args": [
				"build",
				"--symlink-install",
				"--event-handlers", 
				"console_cohesion+",
				"--base-paths",
				"/home/<user_name>/WhiteBox-PC-BOT-rpi4/src", // Change the path to your src folder path
				"--cmake-args",
				"-DCMAKE_BUILD_TYPE=debug" // Can change CMake build type(Debug, Release, RelWithDebInfo and MinSizeRel)
			],
			// Run the command from the src folder
			"options": {
				"cwd": "${workspaceFolder}/src"
			},
			"problemMatcher": [
				"$catkin-gcc"
			],
			"group": {
				"kind": "build",
				"isDefault": true
			},
			"label": "colcon: build"
		}
	]
}
```

If it is the default task,it can be run by ctrl + shift + b
