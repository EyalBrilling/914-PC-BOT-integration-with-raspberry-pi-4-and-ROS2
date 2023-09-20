# Developer guide

## Creating new nodes

## Working on the robot driver

The new driver in [/wbr914_base_driver/](/wbr914_base_driver/)  doesn't implement all of the original robot driver functions in [/wbr914_base_driver/player_driver/](/wbr914_base_driver/player_driver/).  
All player structs and functions used are gone.  

There are a few functions that can be added in the future to the new driver based on the player driver, noticablly IR and analog functions that the robot gives but right now aren't used.

If you want to implement those, in the cpp file of the old driver there is documntation for each function - if it is used or not in the new driver.
