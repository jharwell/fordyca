# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

# Papers

1. J. Harwell and M. Gini, "Broadening applicability of swarm-robotic foraging
   through constraint relaxation," 2018 IEEE International Conference on
   Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR), Brisbane,
   Australia, 2018, pp. 116-122.
   [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8376280&isnumber=8376259)

# Pre-cloning Setup

1. Install development packages for the project:

   - Qt5 (`qtbase5-dev` on ubuntu)
   - NLopt (`libnlopt-dev` on ubuntu)
   - ccache (`ccache` on ubuntu)

2. Install ARGoS (homepage http://www.argos-sim.info/index.php), the simulator
   for the project. You can install it in one of three ways:

   - Use one of the pre-packaged/binary versions of ARGoS from the website (not
     recommended). If you do choose this route then you _MUST_ also use gcc/g++
     version < 6.0 (anything 5.4 is known to work) on linux. This is because
     those packages were compiled with gcc/g++ 5.4, and therefore the core ARGoS
     libraries that fordyca uses are ABI incompatible with anything compiled
     with gcc >= 6.0. In addition, when installing ARGoS from a .deb you will
     likely not have all dependencies met (dpkg does not check them like apt
     does), so you need to run:

        sudo apt install -f

      After installing the .deb with dpkg to fix installation issues.

    - Build upstream ARGoS (https://github.com/ilpincy/argos3) from source. If
      using this method you can use whatever compiler/compiler version you like,
      so long as it supports C++14. This method is generally OK, though you will
      not get some of the tweaks/improvements to ARGoS that I have made that
      have not made their way into the main ARGoS repository yet.

    - Build the organization's downstream ARGoS
      (https://github.com/swarm-robotics/argos3) from source. If
      using this method you can use whatever compiler/compiler version you like,
      so long as it supports C++14. This method is recommended, as you will
      always have the most up-to-date ARGoS functionality (origin+my changes).

3. Verify that you can build an run the ARGoS examples
   (https://github.com/ilpincy/argos3-examples), especially the foraging
   example, which is what the project was originally based on

4. This project uses the build scaffolding provided by
   [libra](https://github.com/swarm-robotics/libra). Please look at the platform
   requirements for that project and install any needed packages/libraries.

5. Clone `rcppsw` https://github.com/swarm-robotics/rcppsw (Reusable
   C++ software) somewhere and create a symbolic link to it under ext/rcppsw:

        mkdir ext
        ln -s /path/to/rcppsw ext/rcppsw

   Follow all pre/post-cloning instructions found in README for the rcppsw repo.

# Post-cloning setup

1. Check out the development branch, as that has not only the latest semi-stable
   release, but also the most up-to-date documentation, including this README.

        git checkout devel

2. Pull in the build scaffolding:

        git submodule update --init --recursive --remote

3. Build via:

        mkdir build && cd build
        cmake ..
        make

   To build the documentation, do the following from the build directory:

        make documentation

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
