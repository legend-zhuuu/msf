#!/usr/bin/env bash

set -ex

GTSAM_VERSION=${1:-"4.0.3"}
NUM_THREADS=${2:-8}


# install gtsam
# cd /tmp && \
#       git clone https://github.com/borglab/gtsam.git && \
#       cd gtsam && \
#       git checkout ${GTSAM_VERSION}

cd /tmp/gtsam

cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release  \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON &&\
  cmake --build build -- -j${NUM_THREADS} && \
  cmake --build build --target install

# clean
cd /tmp &&\
    rm -rf gtsam