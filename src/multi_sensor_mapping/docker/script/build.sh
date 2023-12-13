#!/usr/bin/env bash

set -x

DEVEL_IMAGE_NAME=${DEVEL_IMAGE_NAME:-slam/mapping:ubuntu20.04}

build_image(){
  local devel_platform=${1:-"linux/amd64"}
  
  if [ "x${devel_platform}" = "xlinux/amd64" ];then
    docker build \
      -t "${DEVEL_IMAGE_NAME}-amd64" \
      -f Dockerfile \
      --platform "${devel_platform}" \
      --load \
      .&>build.log
  elif [ "x${devel_platform}" = "xlinux/arm64/v8" ];then
    docker build \
      -t "${DEVEL_IMAGE_NAME}-arm64v8" \
      -f Dockerfile.arm64v8 \
      --platform "${devel_platform}" \
      --load \
      .&>build.log
  else 
    echo "Unknown platform:${devel_platform}"
  fi
}


# build_image "linux/amd64" # amd64
build_image "linux/arm64/v8" # arm64v8

exit 0