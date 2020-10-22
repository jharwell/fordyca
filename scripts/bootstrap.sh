#!/bin/bash
#
# Bootstraps the project, including:
#
# - Install .deb dependencies from repositories (ubuntu assumed)
# - Install python package dependencies
# - Build and install ARGoS from source
# - Set up dependent repositories
# - Compile main project
#
# Assumes sudo privileges
#
# $1 - The root directory for all repos for the project
# $2 - Install system packages? [YES/NO]
# $3 - Should ARGoS be installed system-wide? [YES/NO]
# $4 - The root directory for ARGoS installation
# $5 - # cores to use when compiling

if [ "$#" -ne 5 ]; then
    echo "FATAL: Wrong number of cmdline arguments supplied--see docs at https://swarm-robotics-fordyca.readthedocs.io/en/latest."
    echo "Usage: ./bootstrap.sh <repo dir> <YES/NO> <YES/NO> <ARGoS install dir> <# cores>"
    exit
fi

repo_root=$1
install_sys_pkgs=$2
argos_sys_install=$3
argos_install_dir=$4
n_cores=$5

mkdir -p $repo_root && cd $repo_root

# Install system packages
if [ "YES" = "$install_sys_pkgs" ]; then
    fordyca_pkgs=(qtbase5-dev libnlopt-dev libnlopt-cxx-dev libfreeimageplus-dev
                  freeglut3-dev libeigen3-dev libudev-dev liblua5.3-dev pip3)
    rcppsw_pkgs=(libboost-all-dev liblog4cxx-dev catch ccache python3-pip)
    libra_pkgs=(make cmake git nodejs npm graphviz doxygen cppcheck cmake make gcc-9 g++-9 
                libclang-9-dev clang-tools-9 clang-format-9 clang-tidy-9)

    # Install packages (must be loop to ignore ones that don't exist)
    for pkg in "${libra_pkgs[@]}" "${rcppsw_pkgs[@]}" "${fordyca_pkgs[@]}"
    do
        sudo apt-get -my install $pkg
    done
fi

# Install python packages for user
python_pkgs=(cpplint breathe)
pip3 install --user  "${python_pkgs[@]}"

# Exit when any command after this fails. Can't be before the package installs,
# because it is not an error if some of the packages are not found (I just put a
# list of possible packages that might exist on debian systems to satisfy
# project requirements).
set -e

# Install ARGoS
if [ -d argos3 ]; then rm -rf argos3; fi
git clone https://github.com/swarm-robotics/argos3.git
cd argos3
mkdir -p build && cd build

git checkout devel
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc-9\
      -DCMAKE_CXX_COMPILER=g++-9\
      -DARGOS_BUILD_FOR=simulator\
      -DARGOS_BUILD_NATIVE=ON\
      -DARGOS_THREADSAFE_LOG=ON\
      -DARGOS_DYNAMIC_LIBRARY_LOADING=ON\
      -DARGOS_USE_DOUBLE=ON\
      -DARGOS_DOCUMENTATION=ON\
      -DARGOS_INSTALL_LDSOCONF=$argos_sys_install \
      -DCMAKE_INSTALL_PREFIX=$argos_install_dir \
	  ../src
make -j $n_cores
make doc

if [ "YES" = "$argos_sys_install" ]; then
    sudo make install;
else
    make install;
fi;

cd ../../

# Bootstrap RCPPSW
if [ -d rcppsw ]; then rm -rf rcppsw; fi
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout devel
git submodule update --init --recursive --remote

cd ..

# Bootstrap COSM
if [ -d cosm ]; then rm -rf cosm; fi
git clone https://github.com/swarm-robotics/cosm.git
cd cosm
git checkout devel
git submodule update --init --recursive --remote

rm -rf ext/rcppsw
ln -s $repo_root/rcppsw ext/rcppsw

cd ..

# Bootstrap FORDYCA
if [ -d fordyca ]; then rm -rf fordyca; fi
git clone https://github.com/swarm-robotics/fordyca.git
cd fordyca
git checkout devel
git submodule update --init --recursive --remote
npm install

rm -rf ext/cosm
ln -s $repo_root/cosm ext/cosm

# Build FORDYCA and documentation
mkdir -p build && cd build
cmake \
    -DCMAKE_C_COMPILER=gcc-9\
    -DCMAKE_CXX_COMPILER=g++-9\
    -DCOSM_PROJECT_DEPS_PREFIX=$argos_install_dir\
    ..
make -j $n_cores
make documentation

cd ../../

# Bootstrap sierra
if [ -d sierra ]; then rm -rf sierra; fi
git clone https://github.com/swarm-robotics/sierra.git
cd sierra
git checkout devel
cd ..

if [ -d sierra-plugin-fordyca ]; then rm -rf sierra-plugin-fordyca; fi
git clone https://github.com/swarm-robotics/sierra-plugin-fordyca.git
cd sierra-plugin-fordyca
git checkout devel

cd ..

ln -s $repo_root/sierra-plugin-fordyca sierra/projects/fordyca

# If installed ARGoS as root, all project repos are also owned by root, so we
# need to fix that.
if [ "$YES" = "$argos_sys_install" ]; then
    sudo chown $SUDO_USER:$SUDO_USER -R $repo_root
fi;

# Made it!
echo "Bootstrap successful!"
