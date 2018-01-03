# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

## Basic Setup

Before starting, make sure you have ARGoS installed, and can run the simple
foraging example given on the website.

These steps are for Linux, and while it may work on OSX, I have not tried it. It
definitely will not work on windows. You will need a recent version of the
following programs:

- cmake
- make
- g++
- cppcheck (static analysis)
- clang-check3.8 (syntax checking/static analysis )
- clang-format-4.0 (automatic code formatting)
- clang-tidy-4.0 (static analysis/automated checking of naming conventions)

In addition, you will possibly want to install these programs:

- ccache (will make compiling a lot faster)
- icpc (additional syntax checking)
- ctags/gtags/rtags/cscope (moving around in a large C/C++ code base)

1. After cloning this repo, you will also need to clone the following repos:

  - https://github.com/jharwell/rcppsw (Reusable C++ software)
  - https://github.com/jharwell/rcsw (Reusable C software)
  - https://github.com/jharwell/devel (dotfiles, project config, templates)

  Before you can build anything, you will need to define some environment
  variables:

  - `rcsw` - Set to the path to wherever you cloned the `rcsw` repo.

  - `rcppsw` - Set to the path to wherever you cloned the `rcppsw` repo.

  - `develroot` - Set to the path to wherever you cloned the `devel` repo.

2. Adjust symlinks, as describe in the RCPPSW [README](https://github.com/jharwell/rcppsw/blob/master/README.md).

3. Verify you can build `rcsw`, `rcppsw`, and `fordyca` (in that order), by
   doing:

        cd /path/to/repo
        mkdir build && cd build
        cmake ..
        make

## Development Guide

See the development guide in RCPPSW: [README](https://github.com/jharwell/rcppsw/blob/master/README.md).
