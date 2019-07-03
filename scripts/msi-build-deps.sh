#!/bin/bash
#
# This script builds the necessar dependencies for the project from scratch,
# targeting the MSI cluster it is run on. You need to source the build
# environment setup file before running this script. MUST be run from an cluster
# node (NOT a login node).
#
# $1 - The directory to clone and build dependencies out of
# $2 - The install prefix (things will be installed to the $MSICLUSTER directory
#      under here).
#
cd $1

# First, NLopt
git clone https://github.com/stevengj/nlopt.git
cd nlopt && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$2/$MSICLUSTER \
      -DNLOPT_PYTHON=NO\
      -DNLOPT_OCTAVE=NO\
      -DNLOPT_MATLAB=NO\
      -DNLOPT_GUILE=NO\
      -DNLOPT_SWIG=NO\
      ..
make -j 8 && make install

cd ../..

# Next, ARGoS
git clone https://github.com/swarm-robotics/argos3.git
cd argos3 && mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$2/$MSICLUSTER \
      -DARGOS_BUILD_FOR=simulator \
      -DARGOS_BUILD_NATIVE=NO \
      -DARGOS_THREADSAFE_LOG=ON \
      -DARGOS_DYNAMIC_LOADING=ON \
      -DARGOS_USE_DOUBLE=ON \
      -DARGOS_INSTALL_LDSOCONF=OFF \
      ../src
make -j 8 && make doc && make install
