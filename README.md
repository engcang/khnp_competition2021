# Korea Hydro & Nuclear Power - Autonomous Robotic Challenge 2021
+ Quadruped robot with manipulator for this year


<br>

#### Please note that the world file is huge, so loading gazebo make take few minutes. (200 seconds for myself with super strong computer)

<br>

## How to use
+ Make sure that you installed `ROS desktop full` version - refer the [wiki page](https://wiki.ros.org/ROS/Installation)
  + It comes with `Qt5`, `Gazebo`, `OpenCV` version 3.2, `cv_bridge`
  + In other words, this repo depends on `QT5`, `Gazebo`, `OpenCV`, `cv_bridge`.

<br>

+ Clone the git
~~~shell
$ cd <your_workspace>/src
$ git clone --recursive https://github.com/engcang/khnp_competition2021
~~~

<br>

+ Add Gazebo Path
~~~shell
$ cd khnp_competition2021/gazebo_map_for_khnp
$ echo "export GAZEBO_MODEL_PATH=:$GAZEBO_MODEL_PATH:$(pwd)/refracted_corridor_map:$(pwd)/rough_terrain_map:$(pwd)/stair_map:$(pwd)/qr_codes:$(pwd)/manipulator_map:$(pwd)/disturbance_map:$(pwd)/common" >> ~/.bashrc
$ . ~/.bashrc
~~~

<br>

+ Build the package
~~~shell
$ cd <your_workspace_where_you_clone_this_repo>
$ catkin build
$ . devel/setup.bash
~~~

<br>

+ Run launch file
~~~shell
$ roslaunch khnp_competition main.launch
~~~

<br>

+ Add your autonomous navigation algorithm code and controller as follows:
  + Open `main.launch` to edit
```xml
<?xml version="1.0"?>
<launch>

<!-- Edit this part with your own algorithms -->



<!-- Do not touch below -->
  <!--  MAIN code -->
  ............

</launch>
```
