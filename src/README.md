## Download ROS2

https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

## Build packages

https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

## Build ROS packages

**To build the ROS packages run the folloing command from the src folder**

```
colcon build
```

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
