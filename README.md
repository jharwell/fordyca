# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

## Pre-cloning Setup

1. Install ARGoS: http://www.argos-sim.info/index.php, the simulator
   for the project.

2. Verify that you can run the simple foraging example that comes
   packaged on the ARGoS website.

3. This project uses the build scaffolding provided by
   [cmake-config](https://github.com/jharwell/cmake-config). Please
   look at the platform requirements for that project and install any
   needed packages/libraries.

4. Install additional development packages for the project:

   - catch (A unit testing framework that some unit tests use).
   - boost (1.58 is known to work; older versions may also work). Install all
     boost libraries via:

        sudo  apt-get install libboost-all-dev

    - Qt (Qt 5 is known to work; older versions may also work). Install Qt5 via:

            sudo apt-get install qtbase5-dev

5. Clone `rcsw` https://github.com/swarm-robotics/rcsw (Reusable C software)
   somewhere and create a symbolic to it in the repo as
   `<repo_root>/ext/rcsw`. Follow all pre/post-cloning instructions found in
   README in that repo.

6. Clone `rcppsw` https://github.com/swarm-robotics/rcppsw (Reusable C++ software)
   somewhere and create a symbolic link it as
   `<repo_root>/ext/rcppsw`. Follow all pre/post-cloning instructions found in
   README for that repo.

## Post-cloning setup

1. Check out the development branch, as that has not only the latest semi-stable
   release, but also the most up-to-date documentation, including this README.

        git checkout devel

2. Pull in the cmake project scaffolding:

        git submodule update --init --recursive

3. Build via:

        mkdir build && cd build
        cmake ..
        make

For additional build configuration options, please see the README in
the [cmake config](https://github.com/jharwell/cmake-config) repo.

To build the documentation, do the following from the build directory:

    make documentation

## Running

To run a simple single source foraging experiment, do the following after a
successful build, from the root of the fordyca repo:

    argos3 -c exp/single-source.argos

which should pop up a nice GUI from which you can start the experiment.

# Troubleshooting

- If you are having trouble building, try:

  1. Updating the cmake submodule:

          git submodule update

  2. Updating `rcppsw` and `rcsw`, and possibly their cmake submodules.

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
