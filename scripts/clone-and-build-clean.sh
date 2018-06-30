#!/bin/bash
#
# This script clones and builds the entire project from scratch. Intended for
# use on MSI. You need to source the build environment setup file before
# running this script.
#
# $1 - The directory to clone and build everything in. Can be relative to
#      current dir, or be specified using and absolute path.
#
mkdir $1 && cd $1
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout devel
git submodule update --init --recursive --remote

cd ..
git clone https://github.com/swarm-robotics/fordyca.git
cd fordyca
git checkout devel
git submodule update --init --recursive --remote
mkdir -p ext
ln -s ../../rcppsw ext/

mkdir build && cd build
cmake -DBUILD_ON_MSI=yes ..
make -j 8
