#!/bin/bash
#
# This script clones the entire project from scratch. Intended for use on
# MSI. You need to source the build environment setup file before running this
# script. This MUST be run from a login node, because you need internet access
# (duh).
#
# $1 - The directory to clone everything into. Can be relative to current dir,
#      or be specified using and absolute path.
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

cd ..
git clone https://github.com/swarm-robotics/sierra.git
cd sierra
git checkout devel
