#!/bin/bash
#
# This script builds the necessary dependencies for the project from scratch,
# targeting the MSI cluster it is run on. You need to source the build
# environment setup file before running this script, AND have already run the
# cloning script. MUST be run from a logine node (NOT a cluster node, because
# those don't always have internet access).
#
# $1 - The directory dependencies were cloned into.
# $2 - The install prefix (things will be installed to the $MSICLUSTER directory
#      under here).
# $3 - How many cores to use when building.
#
set -e
cd $1

# First, NLopt
cd nlopt && mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$2/$MSICLUSTER \
      -DNLOPT_PYTHON=NO\
      -DNLOPT_OCTAVE=NO\
      -DNLOPT_MATLAB=NO\
      -DNLOPT_GUILE=NO\
      -DNLOPT_SWIG=NO\
      ..
make -j $3 && make install

cd ../..

# Next, ARGoS
cd argos3
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$2/$MSICLUSTER \
      -DARGOS_BUILD_FOR=simulator \
      -DARGOS_BUILD_NATIVE=NO \
      -DARGOS_THREADSAFE_LOG=ON \
      -DARGOS_DYNAMIC_LOADING=ON \
      -DARGOS_USE_DOUBLE=ON \
      -DARGOS_INSTALL_LDSOCONF=OFF \
      -DARGOS_WITH_LUA=OFF\
      ../src
make -j $3 && make doc && make install
