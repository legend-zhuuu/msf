#!/usr/bin/env bash

set -ex

NUM_THREADS=${1:-8}

# install CERES-SOLVER
# cd /tmp && \
#     git clone https://github.com/sotex/geographiclib.git && \
#     cd geographiclib  
cd /tmp/geographiclib

# build cmake
cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
  cmake --build build -- -j${NUM_THREADS} && \
  cmake --build build --target install 

# clean
cd /tmp &&\
  rm -rf geographiclib