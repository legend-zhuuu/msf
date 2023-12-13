#!/usr/bin/env bash

set -ex

NUM_THREADS=${1:-8}

# install zipper
# cd /tmp && \
#       git clone https://github.com/lecrapouille/zipper.git --recursive && \
#       cd zipper 

# build zipper
cd /tmp/zipper &&
  make compile-external-libs && \
  make -j${NUM_THREADS} && \
  make install

# clean
cd /tmp &&\
  rm -rf zipper