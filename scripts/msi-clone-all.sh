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
mkdir -p $1 && cd $1

# Clone RCPPSW
if [ -d rcppsw ]; then rm -rf rcppsw; fi
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout devel
git submodule update --init --recursive --remote
cd ..


# Clone COSM
if [ -d cosm ]; then rm -rf cosm; fi
git clone https://github.com/swarm-robotics/cosm.git
cd cosm
git checkout devel
git submodule update --init --recursive --remote

ln -s ../../rcppsw ext/

cd ..


# Clone FORDYCA
if [ -d fordyca ]; then rm -rf fordyca; fi
git clone https://github.com/swarm-robotics/fordyca.git
cd fordyca
git checkout devel
git submodule update --init --recursive --remote

ln -s ../../rcppsw ext/
ln -s ../../cosm ext/

mkdir -p build

cd ..

# Clone SIERRA
if [ -d sierra ]; then rm -rf sierra; fi
git clone https://github.com/swarm-robotics/sierra.git
cd sierra
git checkout devel
