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
   `<fordyca_root>/ext/rcsw`. Follow all pre/post-cloning instructions found in
   README in that repo.

6. Clone `rcppsw` https://github.com/swarm-robotics/rcppsw (Reusable C++ software)
   somewhere and create a symbolic link it as
   `<fordyca_root>/ext/rcppsw`. Follow all pre/post-cloning instructions found in
   README for that repo.

## Post-cloning setup

1. Pull in the cmake config:

        git submodule update --init --recursive

2. Build via:

        mkdir build && cd build
        cmake ..
        make

# Troubleshooting

- If you are having trouble building, try:

  1. Updating the cmake submodule:

          git submodule update

  2. Updating `rcppsw` and `rcsw`, and possibly their cmake submodules

  If the problem perists, open an issue.

- If you are getting a segfault when running ARGoS, verify that if you are
  running with Qt visualizations that the threadcount is 0 (Qt5 cannot run with
  multiple threads without segfaulting).

## Contributing

For contributing to `fordyca`, see
[CONTRIBUTING](https://github.com/swarm-robotics/rcppsw/blob/master/docs/CONTRIBUTING.md).
