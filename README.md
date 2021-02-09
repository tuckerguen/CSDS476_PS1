# CSDS 476 - PS1
## Tucker Guen
### Description
This package uses a dead reckoning method to guide the robot from stdr_launchers
server_with_map_and_gui_plus_robot.launch from the the start position to the top left of the map.
### Use
In terminal:
1. Launch the stdr map and robot
    ```{bash}
    > roslaunch stdr_launchers server_with_map_and_gui.launch
    ```
2. Run the dead reckoning open loop commander
    ```{bash}
   > rosrun my_stdr_control my_stdr_control_node 
   ```