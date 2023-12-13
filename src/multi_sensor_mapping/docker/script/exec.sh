#!/usr/bin/env bash

set -x

SLAM_CONTAINER_NAME=${SLAM_CONTAINER_NAME:-mapping}

docker start "${SLAM_CONTAINER_NAME}"
xhost +local:docker
docker exec -it "${SLAM_CONTAINER_NAME}" /bin/bash
