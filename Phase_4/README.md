# Planning Project 3 Phase 4
- Implementation of A star algorithm on Turtlebot in Gazebo Simulator

## Authors
- Achal Vyas
- Pruthvi Sanghavi

## Dependencies
- Gazebo 7._
- ROS Kinetic
- Ubunutu 16
- Git

## Install turtlebot packages
Open a terminal and type...
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
```

## Build Instructions

### From Repository
- Open the terminal and type in the following commands.
```
mkdir <workspace>
cd workspace
mkdir src
cd src
git clone https://github.com/Achalpvyas/Project3.git
cd ..
catkin_make
source devel/setup.bash
```

### From Compressed zip package
- Create a workspace folder.
- Extract the zip in the <workspace> folder
- Open terminal and enter the workspace folder,
```
cd <workspace>
catkin_make
source devel/setup.bash
```

## Run Instructions
- Open terminal and type...
- values in the ```roslaunch``` arguments are float characters.
```
cd <workspace>
roslaunch pathplan turtle_botlauncher.launch x:= <x coordinate> y:= <y coordinate> z:=<z coordinate>
```

## Results