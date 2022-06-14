#!/bin/bash
docker run -it \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-h $HOSTNAME -v $HOME/.Xauthority:/home/hlm/.Xauthority \
-v "$(pwd)"/app:/app \
--network host \
--rm ros:noetic-robot
