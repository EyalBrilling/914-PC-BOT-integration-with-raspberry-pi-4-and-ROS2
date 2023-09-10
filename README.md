# WhiteBox-PC-BOT-rpi4
Using the PC-BOT with raspberry pi 4 + ROS

## WhiteBox-PC-BOT dropbox

https://www.dropbox.com/home/WhiteBox-PC-Bot

## Downloading Player

https://github.com/playerproject/player


## Add the project path as an ENV variable

Add the folloing line to your .bashrc file:

```bash
export WBR914_PROJECT_PATH="/path/to/your/project"
```

## Example adding files to compile in ROS

https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

## Build ROS packages

**To build the ROS packages run the following command from the src folder**

```
colcon build
```


## open() Premission denied

Make sure user is part of the dialout group on linux by:

```shell
groups your_username
```

If not,add yourself to it:

```shell
sudo usermod -aG dialout <your_username>
```