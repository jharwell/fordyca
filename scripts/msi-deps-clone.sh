#!/bin/bash
usage() {
    cat << EOF >&2
Usage: $0 [--rroot <dir>]

--rroot <dir>: The root directory for all repos for the project. All github
               repos will be cloned/built in here. Default=$HOME/research.

-h|--help: Show this message.

This script clones the necessary dependencies for the project from
scratch, targeting the MSI cluster it is run on. MUST be run from a
login node (NOT a cluster node).

EOF
    exit 1
}

if [ -z "${SWARMROOT}" ]; then
    . /home/gini/shared/swarm/bin/msi-env-setup.sh
fi

REPO_ROOT=$HOME/research
options=$(getopt -o h --long rroot:,help  -n "DEPS CLONE" -- "$@")
if [ $? != 0 ]; then usage; exit 1; fi

eval set -- "$options"
while true; do
    case "$1" in
        -h|--help) usage;;
        --rroot) REPO_ROOT=$2; shift;;
        --) break;;
        *) break;;
    esac
    shift;
done

echo -e "********************************************************************************"
echo -e "DEPS CLONE START:\n"
echo -e "REPO_ROOT=$REPO_ROOT\n"
echo -e "********************************************************************************"

set -e
mkdir -p $REPO_ROOT && cd $REPO_ROOT

# First, NLopt
if [ -d nlopt ]; then rm -rf nlopt; fi
git clone https://github.com/stevengj/nlopt.git

# Next, ARGoS
if [ -d argos3 ]; then rm -rf argos3; fi
git clone https://github.com/swarm-robotics/argos3.git
cd argos3
git checkout devel
cd ..

# Made it!
echo -e "********************************************************************************"
echo -e "DEPS CLONE SUCCESS!"
echo -e "********************************************************************************"
