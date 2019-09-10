#!/bin/bash
#
# This script builds the entire project from scratch. Intended for
# use on MSI. You need to source the build environment setup file before
# running this script. MUST be run from an cluster node (NOT a login node).
#
# $1 - The directory that everything was cloned into. Can be relative to current
#      dir, or be specified using and absolute path.
# $2 - How many cores to use when building
#
cd $1/fordyca/build
cmake -DWITH_FOOTBOT_BATTERY=NO -DWITH_FOOTBOT_RAB=NO -DWITH_FOOTBOT_LEDS=NO -DCMAKE_BUILD_TYPE=OPT -DLIBRA_ER=NONE -DBUILD_FOR_MSI=YES  ..
make -j $2
