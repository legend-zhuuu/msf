#!/usr/bin/env bash

# IMAGE_TO_RUN=${IMAGE_NAME:-slam/mapping:ubuntu18.04-amd64} # amd64
IMAGE_TO_RUN=${IMAGE_NAME:-slam/mapping:ubuntu20.04-arm64v8} # amd64

docker stop mapping || true
docker rm mapping || true

xhost +local:docker
docker run -it -d \
  --privileged \
  --name mapping \
  --ipc=host \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${PWD}/../../../:/workspace/msm_ws/" \
  --volume="/dev:/dev" \
  --cap-add=SYS_PTRACE --security-opt seccomp=unconfined \
  ${IMAGE_TO_RUN}
