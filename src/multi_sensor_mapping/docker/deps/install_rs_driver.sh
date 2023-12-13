#!/usr/bin/env bash

set -ex

RS_DRIVER_VERSION=${1:-"v1.5.8"}
NUM_THREADS=${2:-8}


# install RS_DRIVER
# cd /tmp && \
#     git clone https://github.com/RoboSense-LiDAR/rs_driver.git && \
#     cd rs_driver && \
#     git checkout ${RS_DRIVER_VERSION}

cd /tmp/rs_driver

# build cmake
cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
  cmake --build build -- -j${NUM_THREADS} && \
  cmake --build build --target install 

# clean
cd /tmp &&\
  rm -rf rs_driver