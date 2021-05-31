#!/bin/bash
#
# Bootstraps the project, including:
#
# - Install .deb dependencies from repositories (ubuntu assumed)
# - Install python package dependencies
# - Build and install ARGoS from source
# - Set up dependent repositories
# - Compile main project

usage() {
    cat << EOF >&2
Usage: $0 --prefix [/usr/local|$HOME/<dir>] [--rroot <dir>] [--cores <n_cores>] [--nosyspkgs ] [--opt] [-h|--help]

--prefix <dir>: The directory to install ARGoS and other project dependencies
                to. To install ARGoS systemwide (and therefore not have to set
                LD_LIBRARY_PATH), pass '/usr/local' (no quotes); this will
                require sudo access. If you are going to be modifying ARGoS at
                all, then you should not use '/usr/local'. Default=$HOME/.local.

--rroot <dir>: The root directory for all repos for the project. All github
               repos will be cloned/built in here. Default=$HOME/research.

--cores: The # cores to use when compiling. Default=$(nproc).

--nosyspkgs: If passed, then do not install system packages (requires sudo
             access). Default=YES (install system packages).

--opt: Perform an optimized build of FORDYCA. Default=NO.

-h|--help: Show this message.
EOF
    exit 1
}

repo_root=$HOME/research
install_sys_pkgs="YES"
prefix=$HOME/.local
n_cores=$(nproc)
build_type="DEV"
options=$(getopt -o h --long help,prefix:,rroot:,cores:,nosyspkgs,opt  -n "BOOTSTRAP" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --prefix) prefix=$2; shift;;
        --rroot) repo_root=$2; shift;;
        --cores) n_cores=$2; shift;;
        --nosyspkgs) install_sys_pkgs="NO";;
        --opt) build_type="OPT";;
        --) break;;
        *) break;;
    esac
    shift;
done

echo -e "********************************************************************************"
echo -e "BOOTSTRAP START:\n"
echo -e "PREFIX=$prefix\nREPO_ROOT=$repo_root\nN_CORES=$n_cores\nSYSPKGS=$install_sys_pkgs\nBUILD_TYPE=$build_type\n"
echo -e "********************************************************************************"

mkdir -p $repo_root && cd $repo_root

# Install system packages
if [ "YES" = "$install_sys_pkgs" ]; then
    fordyca_pkgs=(qtbase5-dev libnlopt-dev libnlopt-cxx-dev libfreeimageplus-dev
                  freeglut3-dev libeigen3-dev libudev-dev liblua5.3-dev pip3 npm)
    rcppsw_pkgs=(libboost-all-dev liblog4cxx-dev catch ccache python3-pip)
    libra_pkgs=(make cmake git nodejs npm graphviz doxygen cppcheck cmake make gcc-9 g++-9
                libclang-9-dev clang-tools-9 clang-format-9 clang-tidy-9)
    sierra_pkgs=(pip3)

    # Install packages (must be loop to ignore ones that don't exist)
    for pkg in "${libra_pkgs[@]}" "${rcppsw_pkgs[@]}" "${fordyca_pkgs[@]}" "${sierra_pkgs[@]}"
    do
        sudo apt-get -my install $pkg
    done
fi

# Install python packages for user
python_pkgs=(cpplint breathe)
pip3 install --user --upgrade pip
pip3 install --user  "${python_pkgs[@]}"

# Exit when any command after this fails. Can't be before the package installs,
# because it is not an error if some of the packages are not found (I just put a
# list of possible packages that might exist on debian systems to satisfy
# project requirements).
set -e

# Install ARGoS
if [ "/usr/local" = "$prefix" ]; then
    argos_sys_install="YES"
else
    argos_sys_install="NO"
fi;

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
      -DCMAKE_INSTALL_PREFIX=$prefix \
	  ../src
make -j $n_cores
make doc

if [ "YES" = "$argos_sys_install" ]; then
    sudo make install;
else
    make install;
fi;

cd ../../

# Install extended E-puck ARGoS model
if [ -d argos3-eepuck3D ]; then rm -rf argos3-eepuck3D; fi
git clone https://github.com/swarm-robotics/argos3-eepuck3D.git
cd argos3-eepuck3D
mkdir -p build && cd build

git checkout devel

# This code expects ARGoS to be installed system wide, so we have to
# tell it that where the necessary ARGoS cmake, pkgconfig, files are.
cmake -DCMAKE_BUILD_TYPE=Release\
      -DCMAKE_CXX_COMPILER=g++-9\
      -DCMAKE_LIBRARY_PATH=$prefix/lib\
      -DCMAKE_PREFIX_PATH=$prefix/include\
      -DCMAKE_MODULE_PATH=$prefix/share\
      -DCMAKE_INSTALL_PREFIX=$prefix\
      ../src

make -j $n_cores

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

rm -rf ext/cosm
ln -s $repo_root/cosm ext/cosm

# Build FORDYCA and documentation
if [ "OPT" = "$build_type" ]; then
    er="NONE"
else
    er="ALL"
fi;

mkdir -p build && cd build

cmake \
    -DCMAKE_C_COMPILER=gcc-9\
    -DCMAKE_CXX_COMPILER=g++-9\
    -DCMAKE_BUILD_TYPE=OPT\
    -DCOSM_PROJECT_DEPS_PREFIX=/usr/local\
    -DLIBRA_ER=NONE\
    ..
make -j $n_cores
make documentation

cd ../../

# Bootstrap sierra
if [ -d sierra ]; then rm -rf sierra; fi
git clone https://github.com/swarm-robotics/sierra.git
cd sierra
git checkout devel
pip3 install --user -r requirements/common.txt
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
echo -e "********************************************************************************"
echo -e "BOOTSTRAP SUCCESS!"
echo -e "********************************************************************************"
