#!/usr/bin/env bash

set -ex

CERES_VERSION=${1:-"2.0.0"}
NUM_THREADS=${2:-8}


# install CERES-SOLVER
# cd /tmp && \
#     git clone https://ceres-solver.googlesource.com/ceres-solver && \
#     cd ceres-solver  && \
#     git checkout ${CERES_VERSION}

cd /tmp/ceres-solver
rm -r .git

cmake -Bbuild -H. -DCMAKE_POSITION_INDEPENDENT_CODE=ON &&\
  cmake --build build -- -j${NUM_THREADS} && \
  cmake --build build --target install

# clean
cd /tmp &&\
    rm -rf ceres-solver