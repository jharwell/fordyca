#!/bin/bash
#
# Do not try to run this script somewhere other than on MSI, as it probably
# will not work.

GINI_ROOT=/home/gini
SWARM_ROOT=$GINI_ROOT/shared/swarm

# Load modules
module load cmake/3.9.3
module load qt/5.9.1
module load boost/1.65.1/gnu-7.2.0
module load gcc/7.2.0
module load llvm/5.0.0
module load python3
module load parallel

# Set compiler vars so that cmake uses the correct version of the
# compiler. You would think that this would not be required...
export CC=gcc
export CXX=g++

# Add argos to our path
gcc_prefix=$(gcc -v 2>&1  |grep prefix | awk -F'=' '{print $2}' | awk '{print $1}')
export PATH=$PATH:$SWARM_ROOT/bin
export ARGOS_PLUGIN_PATH=$SWARM_ROOT/lib/argos3
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SWARM_ROOT/lib/argos3:$gcc_prefix/lib64
