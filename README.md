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

In addition, you will possibly want to install these programs:

- ccache (will make compiling a lot faster)
- icpc (additional syntax checking)
- ctags/gtags/rtags/cscope (moving around in a large C/C++ code base)

1. After cloning this repo, you will also need to clone the following repos:

  - https://github.com/jharwell/devel (dotfiles, project config, templates)

  Before you can build anything, you will need to define some environment
  variables:

  - `develroot` - Set to the path to wherever you cloned the `devel` repo.

2. Adjust symlinks, as describe in the RCPPSW [README](https://github.com/jharwell/rcppsw/blob/master/README.md).

3. Verify you can build, by doing:

        cd /path/to/repo
        mkdir build && cd build
        cmake ..
        make

## Development Guide

You will also want to clone `rcsw`, `rcppsw` somewhere and link it into
`ext/rcsw` and `ext/rcppsw` respectively, rather than having cmake clone them,
so that if you make changes to it they will be reflected in the code you are
building.

  - https://github.com/jharwell/rcsw (Reusable C software)
  - https://github.com/jharwell/rcppsw (Reusable C++ software)

See the development guide in RCPPSW: [README](https://github.com/jharwell/rcppsw/blob/master/README.md).
