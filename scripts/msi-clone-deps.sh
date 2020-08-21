#!/bin/bash
#
# This script clones the necessary dependencies for the project from scratch,
# targeting the MSI cluster it is run on. You need to source the build
# environment setup file before running this script. MUST be run from an cluster
# node (NOT a login node).
#
# $1 - The directory to clone dependencies into.
#
set -e
cd $1

# First, NLopt
if [ -d nlopt ]; then rm -rf nlopt; fi
git clone https://github.com/stevengj/nlopt.git

# Next, ARGoS
if [ -d argos3 ]; then rm -rf argos3; fi
git clone https://github.com/swarm-robotics/argos3.git
cd argos3
git checkout devel
cd ..
