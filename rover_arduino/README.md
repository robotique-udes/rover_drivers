## Dependencies
This arduino package is compiled and managed with PlatFormIO (extension in VSCode). The

## Including custom messages on Arduino using VSCode

1. Make sure that the folder containing your custom messages is in your catkin_ws
2. Build your workspace using `catkin_make`
3. Launch `roscore`
4. Delete `ros_lib` located in `lib`
5. Run `rosrun rosserial_arduino make_libraries.py "path_to_the_library_destination"` (for VSCode, it's your project's "lib" folder)
6. Within VSCode, add `#include <name_of_the_folder_containing_your_msg/custom_msg_file.h>`

## Running the code in ROS
`roslaunch rover_control ptu.launch`

or

```
roscore
rosrun rosserial_arduino serial_node.py /dev/ttyACM1
```
Make sure to connect to the right port.

Add info about launching in