#!/bin/bash
#
# Bootstraps the project for CI, including:
#
# - Install .deb dependencies from repositories (ubuntu assumed)
# - Build and install ARGoS from source
# - Set up dependent repositories
#
# Assumes sudo privileges
#
# $1 - # cores to use when compiling

fordyca_pkgs=(qtbase5-dev libceres-dev libfreeimageplus-dev freeglut3-dev libeigen3-dev)
rcppsw_pkgs=(libboost-all-dev liblog4cxx-dev catch ccache)
libra_pkgs=(graphviz doxygen cppcheck cmake make gcc-7 libclang-6.0-dev
           clang-tools-6.0 clang-format-6.0 clang-tidy-6.0)

# Install packages
sudo apt-get install "${libra_pkgs[@]}" "${rcppsw_pkgs[@]}" "${fordyca_pkgs[@]}" 2>&1 > /dev/null

# Install ARGoS
git clone https://github.com/swarm-robotics/argos3.git
cd argos3
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DARGOS_BUILD_FOR=simulator\
      -DARGOS_BUILD_NATIVE=ON\
      -DARGOS_THREADSAFE_LOG=ON\
      -DARGOS_DYNAMIC_LIBRARY_LOADING=ON\
      -DARGOS_USE_DOUBLE=ON\
      -DARGOS_DOCUMENTATION=ON\
      -DARGOS_INSTALL_LDSOCONF=YES \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++ \
	  ../src

make -j $1 2>&1 > /dev/null
make doc 2>&1 > /dev/null
sudo make install 2>&1 > /dev/null
cd ../../

# Bootstrap rcppsw
cd ext
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout devel
git submodule update --init --recursive --remote
cd ../../

# Bootstrap fordyca
mkdir -p build
