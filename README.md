# Readme

## Brief introduction
This project is a ROS source code, runing in our movingCatch robot. The robot is designed
to achieve the goal of self-moving to a destination and catch a specified box with
different colors and sharps, all of those behaviors rely on the visual system.

Now a simple function have been realized, including smoothing moving, avoiding obstacles,
path planning, mapping, slam, and also catching with vision identification. But there
are still many parts of functions not been achieved. We need a overall vision to find
the goal place not only the catch vision, this is what we will do in the next.

## Picture
(I will put it later)

## Using
Putting this project into `~/catkin_ws/src/`, you can change `catkin_ws` with your ros
root project. Moving `catkin_make.sh` into `~/catkin_ws`.
When you make your project, run `./catkin_make.sh` in `~/catkin_ws` directory, if you
only to compile some section of function, you can run `./catkin_make.sh [function part]`
in `~/catkin_ws` directory, such as `./catkin_make.sh rplidar_ros` to compile rplidar
laser driver code.

## Thanks
As you see, some of code is fork from other repoes, I will write down here and thans to
help.

    - dashgo:   fork from EAI dashgo opensource code: [https://github.com/EAIBOT/dashgo](https://github.com/EAIBOT/dashgo)
    - navigation:   fork from ros: [https://ros-planning/navigation](https://ros-planning/navigation)
    - openslam_gmapping:    fork from ros: [https://ros-perception/openslam_gmapping](https://ros-perception/openslam_gmapping)
    - slam_gmapping:    fork from ros: [https://ros-perception/slam_gmapping](https://ros-perception/slam_gmapping)
    - rplidar_ros:      fork from rplidar opensource code: [https://robopeak/rplidar_ros](https://robopeak/rplidar_ros)

