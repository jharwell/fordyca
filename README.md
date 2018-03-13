# FORDYCA (FOraging Robots use DYnamic CAches)

This is the main entry point for getting started on the project.

## Pre-cloning Setup

Before starting, make sure you have ARGoS installed, and can run the simple
foraging example given on the website.

These steps are for Linux, and while it may work on OSX, I have not tried it. It
definitely will not work on windows. You will need a recent version of the
following programs:

- cmake
- make
- gcc
- g++ A version that supports C++14 is required. A version that supports OpenMP
  is highly recommended, otherwise simulations of large numbers of robots will
  be (much) slower.

You will also need recent versions of the following libraries:

- boost (1.58 is known to work; older versions may also work). Install all boost
  libraries via:

        sudo  apt-get install libboost-all-dev

- Qt (Qt 5 is known to work; older versions may also work). Install Qt5 via:

        sudo apt-get install qtbase5-dev

In addition, you will possibly want to install these programs:

- ccache (will make compiling a lot faster)
- icpc (additional syntax checking comes from Intel Parallel Studio, which is
  ~14GB)
- ctags/gtags/rtags/cscope (moving around in a large C/C++ code base)

## Post-cloning setup

After cloning this repo, you will need to:

1. Pull in the cmake config:

        git submodule update --init --recursive

2. Clone `rcsw` https://github.com/jharwell/rcsw (Reusable C software) somewhere
   and create a symbolic to it in the repo as
   `<path/to/fordyca/>/ext/rcsw`. Follow post-cloning instructions found in
   README in that repo.

3. Clone `rcppsw` https://github.com/jharwell/rcppsw (Reusable C++ software)
   somewhere and create a symbolic link it as
   `<path/to/fordyca>/ext/rcppsw`. Follow post-cloning instructions found in
   README for that repo.

3. Then you can build via:

        mkdir build && cd build
        cmake ..
        make

# Troubleshooting

- If you are having trouble building, try:

  1. Updating the cmake submodule:

          git submodule update

  2. Updating rcppsw and rcsw, and possibly their cmake submodules

  If the problem perists, open an issue.

- If you are getting a segfault when running ARGoS, verify that if you are
  running with Qt visualizations that the threadcount is 0 (Qt5 cannot run with
  multiple threads without segfaulting).

## Contributing

We use the same contributing guidelines as RCPPSW, whose guide can be found
[here](https://github.com/jharwell/rcppsw/blob/master/docs/CONTRIBUTING.md).
