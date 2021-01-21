#!/bin/bash
usage() {
    cat << EOF >&2
Usage: $0 [--rroot <dir>] [--cores <n_cores>]

--rroot <dir>: The root directory for all repos for the project.
               Default=$HOME/research/$MSIARCH.

--cores: The # cores to use when compiling. Default=$(nproc).

-h|--help: Show this message.

This script builds the entire FORDYCA project from scratch. Intended
for use on MSI. MUST be run from a cluster node (NOT a login node).

EOF
    exit 1
}
if [ -z "${SWARMROOT}" ]; then
    . /home/gini/shared/swarm/bin/msi-env-setup.sh
fi

REPO_ROOT=$HOME/research/$MSIARCH
N_CORES=$(nproc)
options=$(getopt -o h --long rroot:,cores:,help  -n "FORDYCA BUILD" -- "$@")
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
echo -e "FORDYCA BUILD START:\n"
echo -e "REPO_ROOT=$REPO_ROOT\nN_CORES=$N_CORES\n"
echo -e "********************************************************************************"

set -e
cd $REPO_ROOT/fordyca/build

cmake -DCMAKE_BUILD_TYPE=OPT\
      -DLIBRA_ER=NONE\
      -DCOSM_BUILD_FOR=MSI\
      ..
make -j $N_CORES

# Made it!
echo -e "********************************************************************************"
echo -e "FORDYCA BUILD SUCCESS!"
echo -e "********************************************************************************"
