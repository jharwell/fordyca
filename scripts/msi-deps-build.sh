#!/bin/bash
#

usage() {
    cat << EOF >&2
Usage: $0 [--rroot <dir>] [--cores <n_cores>]

--rroot <dir>: The root directory for all repos for the project. All github
               repos will be cloned in here. Default=$HOME/research/$MSIARCH.

--cores: The # cores to use when compiling. Default=$(nproc).

-h|--help: Show this message.

This script builds the necessary dependencies for the project from
scratch, targeting the MSI cluster it is run on. You need to have
already run the cloning script. MUST be run from a cluster node.

EOF
    exit 1
}

if [ -z "${SWARMROOT}" ]; then
    . /home/gini/shared/swarm/bin/msi-env-setup.sh
fi

REPO_ROOT=$HOME/research/$MSIARCH
N_CORES=$(nproc)
options=$(getopt -o h --long rroot:,cores:,help  -n "DEPS BUILD" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --rroot) REPO_ROOT=$2; shift;;
        --cores) N_CORES=$2; shift;;
        --) break;;
        *) break;;
    esac
    shift;
done

echo -e "********************************************************************************"
echo -e "DEPS BUILD START:\n"
echo -e "REPO_ROOT=$REPO_ROOT\nN_CORES=$N_CORES\n"
echo -e "********************************************************************************"


set -e
cd $REPO_ROOT

# First, NLopt
cd nlopt
rm -rf build
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$SWARMROOT/$MSIARCH \
      -DNLOPT_PYTHON=NO\
      -DNLOPT_OCTAVE=NO\
      -DNLOPT_MATLAB=NO\
      -DNLOPT_GUILE=NO\
      -DNLOPT_SWIG=NO\
      ..
make -j $N_CORES && make install

cd ../..

# Next, ARGoS
cd argos3
rm -rf build
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=$SWARMROOT/$MSIARCH \
      -DARGOS_BUILD_FOR=simulator \
      -DARGOS_BUILD_NATIVE=NO \
      -DARGOS_THREADSAFE_LOG=ON \
      -DARGOS_DYNAMIC_LOADING=ON \
      -DARGOS_USE_DOUBLE=ON \
      -DARGOS_INSTALL_LDSOCONF=OFF \
      -DARGOS_WITH_LUA=OFF\
      ../src

make -j $N_CORES && make doc && make install
ln -s $SWARMROOT/bin/argos3-$MSIARCH $SWARMROOT/$MSIARCH/bin/argos3

# Next, ARGoS epuck
cd argos3-eepuck3D
rm -rf build
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release\
      -DCMAKE_LIBRARY_PATH=$SWARMROOT/$MSIARCH/lib\
      -DCMAKE_PREFIX_PATH=$SWARMROOT/$MSIARCH/include\
      -DCMAKE_MODULE_PATH=$SWARMROOT/$MSIARCH/share\
      -DCMAKE_INSTALL_PREFIX=$SWARMROOT/$MSIARCH\
      ../src

make -j $N_CORES && make install

cd ../..

echo -e "********************************************************************************"
echo -e "DEPS BUILD SUCCESS!"
echo -e "********************************************************************************"
