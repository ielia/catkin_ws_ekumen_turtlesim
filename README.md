# Ekumen TurtleSim Challenge

## Pre-requisites
* Having ROS Indigo or above installed.
* Having ROS top-level environment variables on the terminals to use.

## Installation
```bash
git clone ${THIS_PACKAGE}
cd catkin_ws_ekumen_turtlesim
catkin_make install
```

## Running

### PolygonActionServer and TurtleSim node
They are both run with a launcher:
```bash
cd ${POLY_CATKIN_WS}
source install/setup.sh
roslaunch ekumen_turtle_commander commander.launch
```

### PolygonActionClient
```bash
cd ${POLY_CATKIN_WS}
source install/setup.sh
rosrun ekumen_turtle_commander polygon_client.py
```

### Setting Turtle Speed in PolygonActionServer
```bash
cd ${POLY_CATKIN_WS}
source install/setup.sh
rosrun ekumen_turtle_commander set_speed.py <linear_speed> <angular_speed>
```

### Pausing/Resuming PolygonActionServer
```bash
cd ${POLY_CATKIN_WS}
source install/setup.sh
```
Pause:
```bash
rosrun ekumen_turtle_commander pause.py
```
Resume:
```bash
rosrun ekumen_turtle_commander resume.py
```

