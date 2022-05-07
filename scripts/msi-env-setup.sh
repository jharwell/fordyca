#!/bin/bash
#
# Do not try to run this script somewhere other than on MSI, as it probably
# will not work.

export GINIROOT=/home/gini
export SWARMROOT=$GINIROOT/shared/swarm

# I should NOT have to do this, but the MSI help staff have not been helpful on
# this at all.
if lscpu | grep -q AuthenticAMD; then
    export MSIARCH=amd
    echo "Environment: AMD EPYC"
else
    export MSIARCH=intel
    echo "Environment: Intel Xeon"
fi

# Add argos to our path
export PATH=$PATH:$SWARMROOT/bin:$HOME/.local/bin
export ARGOS_PLUGIN_PATH=$SWARMROOT/$MSIARCH/lib/argos3
export LD_LIBRARY_PATH=$SWARMROOT/$MSIARCH/lib/argos3:$SWARMROOT/$MSIARCH/lib64

echo $ARGOS_PLUGIN_PATH
echo $LD_LIBRARY_PATH

# Clear all loaded modules so we start from a known empty state
module purge

# Load modules
module load cmake/3.21.3
module load qt/5.9.1
module load boost/1.72.0/gnu-9.2.0
module load gcc/9.2.0
module load llvm/5.0.0
module load python/3.9.3_anaconda2021.11_mamba
module load parallel
module unload gcc/6.1.0

# Set compiler vars so that cmake uses the correct version of the
# compiler. You would think that this would not be required...
export CC=gcc
export CXX=g++

# This needs to be AFTER loading modules, so we do this for the right
# version of GCC. This is needed so glibc is found by the loader.
gcc_prefix=$(gcc -v 2>&1  |grep prefix | awk -F'=' '{print $2}' | awk '{print $1}')
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$gcc_prefix/lib64

# Always generate core dumps if they happen
ulimit -c unlimited
