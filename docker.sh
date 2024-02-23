#!/bin/sh

NAME=ros2_car

xhost + > /dev/null
touch .bash_history

if ! docker container inspect "$NAME" 2> /dev/null > /dev/null; then
  docker run \
    -v "$PWD:/ws/src" \
    -v "$PWD/.bash_history:/root/.bash_history" \
    -v "/tmp:/tmp" \
    -w "/ws" \
    -e DISPLAY \
    --network host \
    --name "$NAME" \
    --rm \
    -d \
    osrf/ros:humble-desktop \
    tail -f /dev/null
fi

docker exec -it "$NAME" /ros_entrypoint.sh bash
