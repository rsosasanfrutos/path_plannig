# path_plannig

This project includes a Docker image  with ubuntu20 and ros noetic, also a shared folder in which the path planning project is.

In order to run the code the Dockerfile should be created with the following command
Then in order to run the docker file you must launch the ./run.sh script

```sh
sudo docker build .

sudo ./run.sh
```

This project gets a list of waypoints from the config file and initializes a Robot
that will follow the path designed by the list of waypoints until the last point 
having an orientation parallel to the current segment of the list of waypoints.

The functioning is to get the nearst waypoint and then follow the list of waypoints
until the end moving always further down the waypoints path.

The steps of the robot are of 10 cm and the acceptable deviation is configurable 
from the launch file.

In order to launch the code you launch the launch file in one terminal inside the docker image
```sh
 cd app/ws/src/fluid_dev_path_plan

 catkin_nake

 source devel/setup.bash

 roslaunch path_plan path_plan.launch
```

And to visualize it the rviz should be launched with the saved config file 

```sh
 rviz -d ws/src/fluid_dev_path_plan/rviz_conf.rviz 
```

In order to run the unit tests run the following command
```sh
 rviz -d ws/src/fluid_dev_path_plan/rviz_conf.rviz 
```
