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
# $2 - Should ARGoS be installed system-wide? [YES/NO]
# $3 - The root directory for ARGoS installation
# $4 - # cores to use when compiling

mkdir -p $1 && cd $1

fordyca_pkgs=(qtbase5-dev libceres-dev npm libfreeimageplus-dev freeglut3-dev libeigen3-dev)
rcppsw_pkgs=(libboost-all-dev liblog4cxx-dev catch ccache pip3)
libra_pkgs=(graphviz doxygen cppcheck cmake make gcc-7 libclang-6.0-dev
           clang-tools-6.0 clang-format-6.0 clang-tidy-6.0)

python_pkgs=(cpplint)

# Install packages
sudo apt-get install "${libra_pkgs[@]}" "${rcppsw_pkgs[@]}" "${fordyca_pkgs[@]}"
sudo -H pip3 install  "${python_pkgs[@]}"

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
      -DARGOS_INSTALL_LDSOCONF=$2 \
      -DCMAKE_INSTALL_PREFIX=$3 \
	  ../src
make -j $4
make doc
if [ "$YES" == "$2" ]; then
    sudo make install;
else
    make install;
fi;

cd ../../

# Bootstrap rcppsw
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout devel
git submodule update --init --recursive --remote
cd ..

# Bootstrap fordyca
git clone https://github.com/swarm-robotics/fordyca.git
cd fordyca
git checkout devel
git submodule update --init --recursive --remote
npm install
ln -s $1/rcppsw ext/rcppsw

# Build fordyca and documentation
mkdir -p build && cd build
cmake ..
make -j $4
make documentation
