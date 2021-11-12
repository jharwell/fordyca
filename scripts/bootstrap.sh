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

--eepuck3D: If passed, install a custom version of the epuck for 3D
            simulations.

--opt: Perform an optimized build of FORDYCA. Default=NO.

-h|--help: Show this message.
EOF
    exit 1
}

# Make sure script was not run as root or with sudo
if [ $(id -u) = 0 ]; then
    echo "This script cannot be run as root."
    exit 1
fi

# For better debugging when the script doesn't work
# set -x

repo_root=$HOME/research
install_sys_pkgs="YES"
prefix=$HOME/.local
n_cores=$(nproc)
build_type="DEV"
eepuck3D="NO"
options=$(getopt -o h --long help,prefix:,rroot:,cores:,nosyspkgs,opt  -n "BOOTSTRAP" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --prefix) prefix=$2; shift;;
        --rroot) repo_root=$2; shift;;
        --cores) n_cores=$2; shift;;
        --eepuck3D) eepuck3D="YES";;
        --nosyspkgs) install_sys_pkgs="NO";;
        --opt) build_type="OPT";;
        --) break;;
        *) break;;
    esac
    shift;
done

################################################################################
# Functions
################################################################################
function bootstrap_fordyca() {
    if [ -d fordyca ]; then rm -rf fordyca; fi
    git clone https://github.com/swarm-robotics/fordyca.git
    cd fordyca
    git checkout devel
    git submodule update --init --recursive --remote

    rm -rf ext/cosm
    mkdir -p ext
    ln -s $repo_root/cosm ext/cosm

    # Build FORDYCA and documentation
    er=$([ "OPT" = "$build_type" ] && echo "NONE" || echo "ALL")

    mkdir -p build && cd build

    cmake \
        -DCMAKE_C_COMPILER=gcc-9\
        -DCMAKE_CXX_COMPILER=g++-9\
        -DCMAKE_BUILD_TYPE=OPT\
        -DCOSM_DEPS_PREFIX=$prefix\
        -DLIBRA_ER=$er\
        ..
    make -j $n_cores
    make documentation

    cd ../../
}

function bootstrap_sierra() {
    if [ -d sierra ]; then rm -rf sierra; fi
    git clone https://github.com/swarm-robotics/sierra.git
    cd sierra
    git checkout devel
    pip3 install --user -r docs/requirements.txt
    cd docs && make man && cd ..
    python3 -m build
    pip3 install .
    cd ..
}

function bootstrap_titerra() {
    if [ -d titerra ]; then rm -rf titerra; fi
    git clone https://github.com/swarm-robotics/titerra.git
    cd titerra
    git checkout devel
    git submodule update --init --recursive --remote
    pip3 install --user -r requirements/common.txt
    cd ..
}

function bootstrap_cosm() {
    wget\
        --no-cache\
        --no-cookies\
        https://raw.githubusercontent.com/swarm-robotics/cosm/devel/scripts/bootstrap.sh\
        -O bootstrap-cosm.sh

    chmod +x bootstrap-cosm.sh
    cosm_syspkgs=$([ "YES" = "$install_sys_pkgs" ] && echo "" || echo "--nosyspkgs")
    cosm_eepuck3D=$([ "NO" = "$eepuck3D" ] && echo "" || echo "--eepuck3D")
    ./bootstrap-cosm.sh \
        --prefix $prefix\
        --rroot $repo_root\
        --cores $n_cores\
        $cosm_eepuck3D\
        $cosm_syspkgs
}

################################################################################
# Bootstrap main
################################################################################

function bootstrap_main() {
    echo -e "********************************************************************************"
    echo -e "FORDYCA BOOTSTRAP START:"
    echo -e "PREFIX=$prefix"
    echo -e "REPO_ROOT=$repo_root"
    echo -e "N_CORES=$n_cores"
    echo -e "SYSPKGS=$install_sys_pkgs"
    echo -e "BUILD_TYPE=$build_type"
    echo -e "********************************************************************************"

    mkdir -p $repo_root && cd $repo_root

    # First, bootstrap COSM
    bootstrap_cosm

    # Install system packages
    if [ "YES" = "$install_sys_pkgs" ]; then
        fordyca_pkgs=(qtbase5-dev
                      libnlopt-dev
                      libnlopt-cxx-dev
                      libfreeimageplus-dev
                      freeglut3-dev
                      libeigen3-dev
                      libudev-dev
                      liblua5.3-dev)
        sierra_pkgs=(pip3)

        # Install packages (must be loop to ignore ones that don't exist)
        for pkg in "${fordyca_pkgs[@]}" "${sierra_pkgs[@]}"
        do
            sudo apt-get -my install $pkg
        done
    fi

    # Exit when any command after this fails. Can't be before the
    # package installs, because it is not an error if some of the
    # packages are not found (I just put a list of possible packages
    # that might exist on debian systems to satisfy project
    # requirements).
    set -e

    # Bootstrap FORDYCA
    bootstrap_fordyca

    # Bootstrap SIERRA
    bootstrap_sierra

    # Bootstrap TITERRA
    bootstrap_titerra

    # Made it!
    echo -e "********************************************************************************"
    echo -e "FORDYCA BOOTSTRAP SUCCESS!"
    echo -e "********************************************************************************"
}

bootstrap_main
