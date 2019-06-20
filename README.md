# EE4308 Project - Micromouse

This project involve controlling a turtlebot to autonomously navigate a maze with an unknown layout and reach the centre.

# Details 

The depth information obtained from the CV images allow the Turtlebot to recognise where the obstacles or walls are and which are subsequently updated in the map. Using that new information and the flood fill algorithm, the robot will autonomously decide on a path that will steadily allow it to reach its goal. Additionally, in order to counter the drift while moving forward, feedback control was added to ensure that the Turtlebot moves in a straight line.

# Installation

Requires ROS

For the CMakeList.txt file, the following lines of code need to be added

find_package(catkin COMPONENTS std_msgs cmd_vel_msgs geometry_msgs message_filters roscpp OpenCV sensor_msgs cv_bridge image_transport )

include_directories(include
     ${catkin_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${OpenCV_INCLUDE_DIRS}
 )
 
add_executable(turtlebot_auto_move_node src/turtlebot_auto_move.cpp)

target_link_libraries(turtlebot_auto_move_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

 For the package.xml, the following dependencies are needed

<build_depend>rospy</build_depend>
<build_depend>roscpp</build_depend>
<build_depend>std_msgs</build_depend>

<run_depend>rospy</run_depend>
<run_depend>roscpp</run_depend>
<run_depend>std_msgs</run_depend>

<buildtool_depend>catkin</buildtool_depend>

#License

[MIT](https://choosealicense.com/licenses/mit/)
