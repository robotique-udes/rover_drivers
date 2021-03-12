# How it works
The way it works can be summarized this way: The arduino will send and/or receive information via topics. Theses topics will be shared via the serial node provided by ROS. Therefore, we need to push the arduino code before launching the ROS code.

## Dependencies
This arduino package is compiled and managed with PlatFormIO (extension to install in VSCode). The [Servo](https://platformio.org/lib/show/883/Servo/installation) library as well as the [Encoder]() library will be needed. 

To install these libraries, go to the platformIO menu in vscode and under the "**libraries**" option

## Including custom messages on Arduino using VSCode

1. Make sure that the folder containing your custom messages is in your catkin_ws
2. Build your workspace using `catkin_make`
3. Launch `roscore`
4. Delete `ros_lib` located in `lib`
5. Run `rosrun rosserial_arduino make_libraries.py "path_to_the_library_destination"` (for VSCode, it's your project's "lib" folder)
6. Within VSCode, add `#include <name_of_the_folder_containing_your_msg/custom_msg_file.h>`

# Pushing your code into the arduino

## 1. Checking Arduino port
Before launching anything, you will need to make sure your arduino is well set up. There are multiple ways to check on which port your arduino is connected. However, to lighten the set up time, two were chosen.
1. Going under the "devices" tab in platformIO (the device should be automatically found and listed)
2. Launching in a terminal (with the arduino unplugged) `ls /dev/ttyACM*`. Nothing should appear. plug the arduino and run the command again. Your device port should now appear.

## 2. Pushing the code
1. Head to the platformIO tab in VScode, then under `megaatmega2560` there should be an `upload` option, which you are going to press.
2. A terminal should pop up and show you if your code compiled and uploaded properly.

# Running the code in ROS

## 1. Method using roslaunch
First and easiest method to run the node will be 

`roslaunch rover_control ptu.launch`

which will launch a serial node attached to a given port (usually /dev/ttyACMX) where X can be found either by
    

## 2. Method using ros commands
Second method is to run `roscore` into a terminal, then doing `ctrl+shift+t` and finally running `rosrun rosserial_arduino serial_node.py /dev/ttyACMX`, where X is the port of your arduino as seen on step #1.