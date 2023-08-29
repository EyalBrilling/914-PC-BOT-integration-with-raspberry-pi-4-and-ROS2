# WhiteBox-PC-BOT-rpi4
Using the PC-BOT with raspberry pi 4 + ROS

## WhiteBox-PC-BOT dropbox

https://www.dropbox.com/home/WhiteBox-PC-Bot

## Downloading Player

https://github.com/playerproject/player


## open() Premission denied

Make sure user is part of the dialout group on linux by:

```shell
groups your_username
```

If not,add yourself to it:

```shell
sudo usermod -aG dialout <your_username>
```