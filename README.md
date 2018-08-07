# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

## Papers

1. J. Harwell and M. Gini, "Broadening applicability of swarm-robotic foraging
   through constraint relaxation," 2018 IEEE International Conference on
   Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR), Brisbane,
   Australia, 2018, pp. 116-122.
   [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8376280&isnumber=8376259)

## Pre-cloning Setup

1. Install development packages for the project:

   - catch (A unit testing framework that some unit tests use).

        sudo apt install catch

   - boost 1.58.

        sudo apt install libboost-all-dev

   - Qt5.

        sudo apt install qtbase5-dev

   - graphviz (For building the documentation include/dependency graphs)

        sudo apt install graphviz

2. Install ARGoS: http://www.argos-sim.info/index.php, the simulator
   for the project.

   *IMPORTANT!* If you use one of the pre-packaged versions of ARGoS, then you
   _MUST_ also use gcc/g++ version < 6.0 (anything 5.4 is known to work) on
   linux. This is because those packages were compiled with gcc/g++ 5.4, and
   therefore the core ARGoS libraries that fordyca uses are ABI incompatible
   with anything compiled with gcc >= 6.0. In addition, when installing ARGoS
   from a .deb you will likely not have all dependencies met (dpkg does not
   check them like apt does), so you need to run:

        sudo apt install -f

   To fix installation issues.

   If you are compiling ARGoS from source you can use whatever compiler/compiler
   version you like, so long as it supports C++14.

3. Verify that you can run the simple foraging example that comes
   packaged on the ARGoS website.

4. This project uses the build scaffolding provided by
   [cmake-config](https://github.com/jharwell/cmake-config). Please
   look at the platform requirements for that project and install any
   needed packages/libraries.


5. Clone `rcppsw` https://github.com/swarm-robotics/rcppsw (Reusable
   C++ software) somewhere and create a symbolic link to it under ext/rcppsw:

        `ln -s /path/to/rcppsw ext/rcppsw`

   Follow all pre/post-cloning instructions found in README for the rcppsw repo.

## Post-cloning setup

1. Check out the development branch, as that has not only the latest semi-stable
   release, but also the most up-to-date documentation, including this README.

        git checkout devel

2. Pull in the cmake project scaffolding:

        git submodule update --init --recursive --remote

3. Build via:

        mkdir build && cd build
        cmake ..
        make

   To build the documentation, do the following from the build directory:

        make documentation

## Running On Your Laptop

After successful compilation, follow these steps to run a foraging scenario:

1. Set the `ARGOS_PLUGIN_PATH` variable to contain the path to the
   `libfordyca.so` file. On bash, that is:

        export ARGOS_PLUGIN_PATH=/path/to/fordyca/build/lib


2. cd to the ROOT of the fordyca repo, and run the experiment:

        argos3 -c exp/single-source.argos

   This should pop up a nice GUI from which you can start the experiment.

## Running on MSI

ARGoS is installed in `/home/gini/shared/swarm`. You should have read/execute
access to that directory as part of the gini group.

1. In `/home/gini/shared/swarm`, source the build/run environment setup
   script:

        . /home/gini/shared/swarm/bin/build-env-setup.sh

   If you use a different shell than bash, you will have to look at the script
   and modify it (in *your* home directory somewhere) so your shell understands
   the syntax.

2. Run the bash script to clone and build the project:

        clone-and-build-clean.sh /path/to/project/root

   The 1st argument is the path (relative or absolute) to the location where you
   want the project repos to live (they will all be cloned into that level).

   If you need to checkout a particular branch in the repo you can do that after
   running the script and then re-running make.

3. Once the script finishes, set `ARGOS_PLUGIN_PATH` as before and you should be
   able to run experiments.

# Troubleshooting

- If you are having trouble building, try:

  1. Verifying that both `fordyca` AND `rcppsw` are on the `devel` branch.

  2. Updating `rcppsw` and `fordyca` to the latest `devel` branch via `git
     pull`.

  2. Updating the `fordyca`, `rcppsw` cmake submodules by running `git submodule
     update` in the root of each repository.


  If the problem perists, open an issue.

- If you are having trouble running experiments (i.e. they won't start/crash
  immediately), try:

  1. If you are getting a segfault when running ARGoS, verify that if you are
     running with Qt visualizations that the threadcount is 0 (Qt5 cannot run
     with multiple threads without segfaulting).

  2. Verify you don't have any anaconda bits in your `PATH`. Depending on
     version, anaconda loads a DIFFERENT version of the Qt than fordyca uses,
     resulting in a dynamic linking error.

## Contributing

For contributing to `fordyca`, see
[CONTRIBUTING](https://github.com/swarm-robotics/rcppsw/blob/master/docs/CONTRIBUTING.md).
