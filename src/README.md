## Download ROS2

https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

## Build packages

https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

## Build wbr914 ROS packages

wbr914 ROS packages are under src.

**To build the ROS packages run the following command from the src folder**

```
colcon build
```

## Run a package executable

After building packages,Running ROS executables is done by running the command:

```shell
ros2 run <package_name> <executable_name>
```

For example,to run wbr914 listener in the `wbr914_velocity_package` package run:

```shell
ros2 run wbr914_velocity_package wbr914_velocity_listener 
```

This command wil run the listener that waits for velocity commands from publishers.


## Compile ROS from Visual studio code

We need to make a new vsc task that will run `colcon build`:

1) If you dont have one,create a new tasks.json file in the .vscode folder.
Can also open vsc command pallete(ctrl+shift+p) and search  "Tasks: Configure Default Build Task" To make the file.

2) Create the colcon build task. Here is a template:

```
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
				"/home/eyalbr/WhiteBox-PC-BOT-rpi4/src",
				"--cmake-args",
				"-DCMAKE_BUILD_TYPE=debug"
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

If it is the default one,it can be run by ctrl + shift + b
