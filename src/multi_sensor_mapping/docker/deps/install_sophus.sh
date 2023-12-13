#!/usr/bin/env bash

set -ex

SOPHUS_VERSION=${1:-"593db47500"}
NUM_THREADS=${2:-8}

# install CERES-SOLVER
# cd /tmp && \
#     git clone https://github.com/strasdat/Sophus.git && \
#     cd Sophus  && \
#     git checkout ${SOPHUS_VERSION} &&\
cd /tmp/Sophus

# build cmake
cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
  cmake --build build -- -j${NUM_THREADS} && \
  cmake --build build --target install 

# clean
cd /tmp &&\
  rm -rf Sophus