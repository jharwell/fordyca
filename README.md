# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

# Papers

1. J. Harwell and M. Gini, "Broadening applicability of swarm-robotic foraging
   through constraint relaxation," 2018 IEEE International Conference on
   Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR), Brisbane,
   Australia, 2018, pp. 116-122.
   [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8376280&isnumber=8376259)

# Setup

Download `scripts/bootstrap.sh` BEFORE cloning this repo, and run it:

    ./bootstrap.sh /path/to/project/root YES /usr/local 4

1st arg is the root directory for the project (all repos will be clone/built in
here), 2nd arg is `YES` if you want to install ARGoS system wide (you probably
do) and `NO` otherwise, 3rd arg is the location ARGoS should be installed into,
and 4th arg is the \# of cores to use when building ARGos/FORDYCA (should be set
to \# cores on your machine). It assumes you have sudo privileges on the machine
you want to install the project on. If you do not, you will have to build a
*lot* more stuff from source manually.

# Configuring Simulations

For parameter configuration see
[parameters](https://github.com/swarm-robotics/fordyca/tree/devel/docs/parameters.md).

## Controller Configuration


| Controller | Status   | Loop functions | Notes                                                                                                                                |
|------------|----------|----------------|--------------------------------------------------------------------------------------------------------------------------------------|
| crw        | Stable   | depth0         | CRW = Correlated Random Walk.                                                                                                        |
| dpo        | Stable   | depth0         | DPO = Mapped Decaying Pheromone Object. Uses pheromones to track objects within the arena.                                           |
| mdpo       | Stable   | depth0         | MDPO = Mapped Decaying Pheromone Object. DPO + mapped extent of the arena tracking relevance of individual cells within it.          |
| gp\_dpo    | Stable   | depth1         | Greedy task partitioning + DPO. Requires static caches to also be enabled.                                                           |
| ogp\_dpo   | Stable   | depth1         | Greedy task partitioning + DPO + oracle (perfect knowledge, as configured). Requires static caches, oracle to be enabled.            |
| grp\_dpo   | Unstable | depth2         | Recursive greedy task partitioning + DPO. Requires dynamic caches to be enabled.                                                     |
| ogrp_dpo   | Unstable | depth2         | Recursive greedy task partitioning + DPO + oracle (perfect knowledge, as configured). Requires dynamic caches, oracle to be enabled. |

# Running On Your Laptop

After successful compilation, follow these steps to run a foraging scenario:

1. Set the `ARGOS_PLUGIN_PATH` variable to contain the path to the
   `libfordyca.so` file. On bash, that is:

        export ARGOS_PLUGIN_PATH=/path/to/where/argos/lib/dir:/path/to/fordyca/build/lib

   Note that you need BOTH of these terms in the path, because this defines the
   ENTIRE search space for argos to look for libraries (including its own core
   libraries).

2. Unless you compile out event reporting, you will need to set the path to the
   log4cxx configuration file. On bash that is:

        export LOG4CXX_CONFIGURATION=/path/to/fordyca/log4cxx.xml

3. cd to the ROOT of the fordyca repo, and run the demo experiment:

        argos3 -c exp/demo.argos

   This should pop up a nice GUI from which you can start the experiment (it
   runs depth0 dpo foraging by default). If no GUI appears, verify that the
   `<visualization>` subtree of the file is not commented out.

# Running on MSI

Head over to [sierra](https://github.com/swarm-robotics-sierra.git), and follow
the MSI setup instructions over there. Don't try to run on MSI without it. Just
don't.

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

For contributing to `fordyca`, see
[CONTRIBUTING](https://github.com/swarm-robotics/rcppsw/blob/master/docs/CONTRIBUTING.md).
