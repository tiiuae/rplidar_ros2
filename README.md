# RPLIDAR ROS package

2D Laser Scanner (LIDAR) ROS node. Broadcasts each scan result into a ROS topic.
Each result conceptually looks like this:

```
       xxxxxx
       x     xxx
       x       xx
       x         x
       x          xx
       x           xxxxxx
       x                xxx
    xxxx                  xx
   xx       o              xx
  xx                        x
 xx                         x
xx                         x
xx    xxxxx               xx
 xxxxxx   xxxx          xxx
             xxxxxxxx  xx
                    xxx
```

Where:

- the `o` is the lidar sensor itself ("origo")
- each `x` is a distance (= a single number) from `o`


## Development, debug

[See documentation in container base image](https://github.com/tiiuae/fog-ros-baseimage/tree/main#development--debug-for-concrete-projects)

## Generic

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

## How to build rplidar ros package
```
1) Clone this project to your catkin's workspace src folder
2) Running catkin_make to build rplidarNode and rplidar_client
```

## How to run rplidar ros package

There're two ways to run rplidar ros package

### I. Run rplidar node and view in the rviz

```
roslaunch rplidar_ros view_rplidar.launch (for RPLIDAR A1/A2)
,
roslaunch rplidar_ros view_rplidar_a3.launch (for RPLIDAR A3)
or
roslaunch rplidar_ros view_rplidar_s1.launch (for RPLIDAR S1)
```

You should see rplidar's scan result in the rviz.

### II. Run rplidar node and view using test application

```
roslaunch rplidar_ros rplidar.launch (for RPLIDAR A1/A2)
,
roslaunch rplidar_ros rplidar_a3.launch (for RPLIDAR A3)
or
roslaunch rplidar_ros rplidar_s1.launch (for RPLIDAR S1)

rosrun rplidar_ros rplidar_client
```

You should see rplidar's scan result in the console

Notice: the different is serial_baudrate between A1/A2 and A3/S1

## RPLidar frame

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png
