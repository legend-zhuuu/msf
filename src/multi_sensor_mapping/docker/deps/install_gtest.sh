#!/usr/bin/env bash

set -ex

# install GTest

cd /usr/src/gtest &&\
  cmake CMakeLists.txt && \
  make -j12 && \
  cp lib/*.a /usr/lib
