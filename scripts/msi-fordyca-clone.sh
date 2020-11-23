#!/bin/bash
usage() {
    cat << EOF >&2
Usage: $0 [--rroot <dir>] [--bbranch <branch>] [--fbranch <branch>]

--rroot <dir>: The root directory for all repos for the project. All github
               repos will be cloned/built in here. Default=$HOME/research.

--bbranch: The branch to checkout in all repos except the FORDYCA
           one. Default=devel.

--fbranch: The branch to checkout in the FORDYCA one. Default=devel.

-h|--help: Show this message.

This script clones the necessary dependencies for the FORDYCA project
from scratch, targeting the MSI cluster it is run on. MUST be run from
a cluster node (NOT a login node).

EOF
    exit 1
}

if [ -z "${SWARMROOT}" ]; then
    . /home/gini/shared/swarm/bin/msi-env-setup.sh
fi

REPO_ROOT=$HOME/research
BASE_BRANCH=devel
FORDYCA_BRANCH=devel

options=$(getopt -o h --long rroot:,bbranch:,fbranch:,help  -n "FORDYCA CLONE" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --rroot) REPO_ROOT=$2; shift;;
        --bbranch) BASE_BRANCH=$2; shift;;
        --fbranch) FORDYCA_ROOT=$2; shift;;
        --) break;;
        *) break;;
    esac
    shift;
done

echo -e "********************************************************************************"
echo -e "FORDYCA CLONE START:\n"
echo -e "REPO_ROOT=$REPO_ROOT\nBASE_BRANCH=$BASE_BRANCH\nFORDYCA_BRANCH=$FORDYCA_BRANCH"
echo -e "********************************************************************************"

mkdir -p $REPO_ROOT && cd $REPO_ROOT

# Clone RCPPSW
if [ -d rcppsw ]; then rm -rf rcppsw; fi
git clone https://github.com/swarm-robotics/rcppsw.git
cd rcppsw
git checkout $BASE_BRANCH
git submodule update --init --recursive --remote
cd ..


# Clone COSM
if [ -d cosm ]; then rm -rf cosm; fi
git clone https://github.com/swarm-robotics/cosm.git
cd cosm
git checkout $BASE_BRANCH
git submodule update --init --recursive --remote

ln -s ../../rcppsw ext/

cd ..


# Clone FORDYCA
if [ -d fordyca ]; then rm -rf fordyca; fi
git clone https://github.com/swarm-robotics/fordyca.git
cd fordyca
git checkout $FORDYCA_BRANCH
git submodule update --init --recursive --remote

ln -s ../../cosm ext/

mkdir -p build

cd ..

# Clone SIERRA
if [ -d sierra ]; then rm -rf sierra; fi
git clone https://github.com/swarm-robotics/sierra.git
cd sierra
git checkout $BASE_BRANCH
git submodule update --init --recursive --remote

cd ..

# Made it!
echo -e "********************************************************************************"
echo -e "FORDYCA CLONE SUCCESS!"
echo -e "********************************************************************************"
