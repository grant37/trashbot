# Trashbot
### Intelligent Autonomous Robotics (Tufts COMP50) Project

###### This project continues to be a work in progress.
To run our current demo, please use the turtlebot workspace. Refernce jsinapov's
(https://github.com/jsinapov/tufts_service_robots:"tufts_service_robots").

### To Run the Demo:

 * You need to be running ROS Indigo.
 * Clone just the 'turtlebot' workspace from this repository.
 * Enter the workspace and make the demo:
 ```
 $ cd ~/turtlebot
 $ catkin_make
 ```
 * Start up amcl navigation:

``` $ roslaunch turtlebot_bringup minimal.launch
$ roslaunch turtlebot_bringup minimal.launch
$ roslaunch turtlebot_navigation amcl_demo.launch map_file:=<full path to your map YAML file>
$ roslaunch turtlebot_rviz_launchers view_navigation.launch
```
* Start up the nav node and waypoint node:
```
$ rosrun trash trash_node
$ rosrun trash waypoint_navigation_node
```

* Start up the object detection node via the python script:

```
$ python ~/turtlebot/src/trashtest/src/trash_test_node.py
```



