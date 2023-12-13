#!/usr/bin/env bash

set -ex


# install CERES-SOLVER
# cd /tmp && \
#   wget https://cmake.org/files/v3.21/cmake-3.21.7-linux-x86_64.tar.gz && \
#   tar -xvzf cmake-3.21.7-linux-x86_64.tar.gz

cd /tmp && \
  mv cmake-3.21.7-linux /opt/cmake-3.21
  ln -sf /opt/cmake-3.21/bin/* /usr/bin

# clean
cd /tmp &&\
    rm -rf cmake-3.21.7-linux