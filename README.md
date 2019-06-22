# FORDYCA (FOraging Robots use DYnamic CAches)

[![Build Status](https://travis-ci.org/swarm-robotics/fordyca.svg?branch=devel)](https://travis-ci.org/swarm-robotics/fordyca.svg?branch=devel)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
![Example Simulation](docs/example-ss.png?raw=true "Example Single Source Foraging Scenario")

This is the main entry point for getting started on the project.

# Papers

1. J. Harwell and M. Gini, "Broadening applicability of swarm-robotic foraging
   through constraint relaxation," 2018 IEEE International Conference on
   Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR), Brisbane,
   Australia, 2018, pp. 116-122.
   [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8376280&isnumber=8376259)

2. J. Harwell and M. Gini, "Swarm Engineering Through Quantitative Measurement
   of Swarm Robotic Principles in a 10,000 Robot Swarm," 2019 Joint Conference
   of Artificial Intelligence (IJCAI), Macau, China, 2019, pp. XXX-XXX.

3. A. Chen, J. Harwell, M. Gini, "Maximizing Energy Efficiency in Swarm
   Robotics," arXiv:1906.01957 [cs.MA], June 2019.
   [Link](https://arxiv.org/abs/1906.01957)

4. N. White, J. Harwell, M. Gini, "Socially Inspired Communication in Swarm
   Robotics," arXiv:1906.01108 [cs.RO], June 2019.
   [Link](https://arxiv.org/abs/1906.01108)

# Setup

Download `scripts/bootstrap.sh` BEFORE cloning this repo. The script can be
downloaded by navigating to the file on github, clicking the `raw` button, and
then right clicking and doing `Save As`. After downloading, mark the script as
executable (`chmod +x bootstrap.sh`) and then run it (it can be run from
anywhere), with the following arguments:

- 1st arg: Is the root directory for the project (all repos will be cloned/built
  in here, and it *must* be an absolute path).
- 2nd arg is `YES` if you want to install ARGoS system wide (you probably do)
  and `NO` otherwise.
- 3rd arg is the location ARGoS should be installed into, and 4th arg is the \#
  of cores to use when building ARGoS/FORDYCA (should be set to \# cores on your
  machine).

For example:

        ./bootstrap.sh $HOME/research YES /usr/local 2

To build the code under `~/research` on a 2 core machine and install ARGoS
system-wide.

The script assumes you have sudo privileges on the machine you want to install
the project on. If you do not, you will have to build a *lot* more stuff from
source manually.

# Available Controllers

| Controller | Status | Loop functions | Notes                                                                                                                                 |
|------------|--------|----------------|---------------------------------------------------------------------------------------------------------------------------------------|
| crw        | Stable | depth0         | CRW = Correlated Random Walk.                                                                                                         |
| dpo        | Stable | depth0         | DPO = Mapped Decaying Pheromone Object. Uses pheromones to track objects within the arena.                                            |
| mdpo       | Stable | depth0         | MDPO = Mapped Decaying Pheromone Object. DPO + mapped extent of the arena tracking relevance of individual cells within it.           |
| odpo       | Stable | depth0         | ODPO = Oracular DPO. Has perfect information about blocks in thye arena.                                                              |
| omdpo      | Stable | depth0         | OMDPO = Oracular MDPO. Has perfect information about blocks in the arena.                                                             |
| gp\_dpo    | Stable | depth1         | Greedy task partitioning + DPO. Requires static caches to also be enabled.                                                            |
| gp\_odpo   | Stable | depth1         | Greedy task partitioning + DPO + oracle (perfect knowledge, as configured). Requires static caches, oracle to be enabled.             |
| gp\_mdpo   | Stable | depth1         | Greedy task partitioning + MDPO. Requires static caches, oracle to be enabled.                                                        |
| gp\_omdpo  | Stable | depth1         | Greedy task partitioning + MDPO + oracle (perfect knowledge, as configured). Requires static caches, oracle to be enabled.            |
| grp\_dpo   | Stable | depth2         | Recursive greedy task partitioning + DPO. Requires dynamic caches to be enabled.                                                      |
| grp\_mdpo  | Stable | depth2         | Recursive greedy task partitioning + MDPO. Requires dynamic caches to be enabled.                                                     |
| grp\_odpo  | Stable | depth2         | Recursive greedy task partitioning + DPO + oracle (perfect knowledge, as configured). Requires dynamic caches, oracle to be enabled.  |
| grp\_omdpo | Stable | depth2         | Recursive greedy task partitioning + MDPO + oracle (perfect knowledge, as configured). Requires dynamic caches, oracle to be enabled. |

# Configuring Simulations

This project extends the base `.argos` file (XML file really) with a set of new
parameters for controllers and loop functions.

For controller configuration, see
[controller](https://github.com/swarm-robotics/fordyca/tree/devel/docs/controller-xml-configuration.md).

For loop functions configuration, see
[loop functions](https://github.com/swarm-robotics/fordyca/tree/devel/docs/loop-functions-xml-configuration.md).

# Running On Your Laptop

After successful compilation, follow these steps to run a foraging scenario:

1. Set the `ARGOS_PLUGIN_PATH` variable to contain the path to the
   `libfordyca.so` file. On bash, that is:

        export ARGOS_PLUGIN_PATH=/path/to/where/argos/lib/dir:/path/to/fordyca/build/lib

   Where you replace both `/path/to` entries to actual paths.  Note that you
   need BOTH of these terms in the path, because this defines the ENTIRE search
   space for argos to look for libraries (including its own core libraries).

2. Unless you compile out event reporting, you will need to set the path to the
   log4cxx configuration file. On bash that is:

        export LOG4CXX_CONFIGURATION=/path/to/fordyca/log4cxx.xml

3. cd to the ROOT of the fordyca repo, and run the demo experiment:

        argos3 -c exp/demo.argos

   This should pop up a nice GUI from which you can start the experiment (it
   runs depth0 dpo foraging by default). If no GUI appears, verify that the
   `<visualization>` subtree of the file is not commented out.

# Running on MSI

Head over to
[sierra](https://github.com/swarm-robotics/sierra/tree/devel/docs/README.md),
and follow the MSI setup instructions over there. Don't try to run on MSI
without it. Just don't.

# Troubleshooting

- If you are having trouble building, try:

  1. Verifying that both `fordyca` AND `rcppsw` are on the `devel` branch.

  2. Updating `rcppsw` and `fordyca` to the latest `devel` branch via `git
     pull`.

  2. Updating the `fordyca`, `rcppsw` cmake submodules by running `git submodule
     update --recursive --remote` in the root of each repository.


  If the problem perists, open an issue.

- If you are having trouble running experiments (i.e. they won't start/crash
  immediately), try:

  1. If you are getting a segfault when running ARGoS, verify that if you are
     running with Qt visualizations that the threadcount is 0 (Qt5 cannot run
     with multiple threads without segfaulting).

  2. Verify you don't have any anaconda bits in your `PATH`. Depending on
     version, anaconda loads a DIFFERENT version of the Qt than fordyca uses,
     resulting in a dynamic linking error.

  3. Make sure you have the necessary environment variables set correctly.

  4. If you get a `std::bad_cast`, `boost::get`, or similar exception, then
     verify that the name of [controller, loop functions, qt user functions],
     are correct, per the table above.

# Contributing

For contributing to `FORDYCA`, see
[CONTRIBUTING](https://github.com/swarm-robotics/rcppsw/tree/devel/docs/CONTRIBUTING.md).
