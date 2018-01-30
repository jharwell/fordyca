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
- gcc
- g++ (a version that supports C++11 is required)

You will also need recent versions of the following libraries:

- boost (1.58 is known to work; older versions may also work). Install all boost
  libraries via:

        sudo  apt-get install libboost-all-dev

- Qt (Qt 5 is known to work; older versions may also work). Install Qt5 via:

        sudo apt-get install qtbase5-dev

In addition, you will possibly want to install these programs:

- ccache (will make compiling a lot faster)
- icpc (additional syntax checking; comes from Intel Parallel Studio, which is ~14GB)
- ctags/gtags/rtags/cscope (moving around in a large C/C++ code base)

After cloning this repo, you will need to:

1. Pull in the cmake config:

        git submodule update --init --recursive

2. Clone `rcsw` https://github.com/jharwell/rcsw (Reusable C software) somewhere
   and link it into `ext/rcsw`.

3. Clone `rcppsw` https://github.com/jharwell/rcppsw (Reusable C++ software)
   somewhere and link it into `ext/rcppsw`.

3. Then you can build via:

        mkdir build && cd build
        cmake ..
        make

## Notes

- ARGoS cannot run under Qt5 with multiple threads without segfaulting, so make
  sure the threadcount is always 0 or 1.

## Contributing

We use the same contributing guidelines as RCPPSW, whose guide can be found
[here](https://github.com/jharwell/rcppsw/blob/master/docs/CONTRIBUTING.md).
