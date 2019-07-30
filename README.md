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

# Setup (Debug build)

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

        ./bootstrap.sh $HOME/research YES /usr/local 2 > output.txt 2>&1

To build the code under `~/research` on a 2 core machine and install ARGoS
system-wide. The `> output.txt 2>&1` part is important to capture the output of
running the script so that if there are errors it is easier to track them down
(the script generates a LOT of output, which usually overflows terminal
ringbuffers).

The script assumes you have sudo privileges on the machine you want to install
the project on. If you do not, you will have to build a *lot* more stuff from
source manually.

*IMPORTANT* If you want to build an _optimized_ version of fordyca (necessary
for large swarms), you will need to either manually modify the `bootstrap.sh`
script that you copied, or re-run `cmake` and `make` as shown below.

# Setup (Optimized Build)

To build forydca with optimizations (necessary for using sierra or running large
scale simulations, will need a different cmake command than the one
`bootstrap.sh` uses for you. Something like the following, run from the `build`
directory prior to building will do the trick:

    cmake -DCMAKE_C_COMPILER=gcc-8 \
    -DCMAKE_CXX_COMPILER=g++-8 \
    -DWITH_FOOTBOT_BATTERY=NO \
    -DWITH_FOOTBOT_RAB=NO \
    -DWITH_FOOTBOT_LEDS=NO \
    -DCMAKE_BUILD_TYPE=OPT \
    -DLIBRA_ER_NREPORT=YES \
    -DLIBRA_OPENMP=YES \
    ..

To get an idea of what some of the non-project specific options mean, head over
to the [libra](https://github.com/swarm-robotics/libra/tree/devel/README.md)
repo and look at the README. *WARNING*: `LIBRA_OPENMP` should only be set to
`YES` if you are going to be running with really large swarms (> 1000),
otherwise the overhead from using threads will probably be greater than any
speedup you will get. YMMV.

`WITH_FOOTBOT_BATTERY`, `WITH_FOOTBOT_RAB`, `WITH_FOOTBOT_LEDS` are things that
are only needed if you are running experiments which utilize those
sensors/actuators, otherwise they slow things down a *LOT* with large swarms
(which is why you are compiling with optimizations on in the first place).

# Viewing The Documentation

After the bootstrap.sh script finishes successfully, you can (*AND SHOULD*) view
the doxygen documentation in your browser by navigating to the generated
`index.html` file. Simply open your browser, and then put the path to the
fordyca repo followed by `/build/docs/html/index.html`. For example, if you
built fordyca under `$HOME/research`, then you would do
`$HOME/research/build/docs/html/index.html` in the address bar of your browser.

Alternatively, if you would like a .pdf of the documentation, you can navigate
to the `latex` directory for doxygen and then build said pdf. Again assuming you
built fordyca in `$HOME/research`, do the following:

    cd $HOME/research/fordyca/build/docs/latex
    make

A `refman.pdf` will (eventually) be built in that directory once the command
finishes. Note that if you want to do build the .pdf you will also need the
following programs:

- `pdflatex` (`texlive-latex-base` on ubuntu)
- Texlive fonts (`texlive-fonts-extra` on ubuntu)

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

1. Set the `ARGOS_PLUGIN_PATH` variable to contain (1) the path to the directory
   containing the `libfordyca.so` file, (2) the path to the ARGoS libraries. On
   bash, that is:

        export ARGOS_PLUGIN_PATH=/usr/local/lib/argos3/lib:$HOME/git/fordyca/build/lib

   Assuming you have installed ARGoS to `/usr/local` and have cloned/built
   fordyca under `$HOME/git`. If your paths are different, modify the above paths
   accordingly. Note that you need BOTH of these terms in the path, because this
   defines the ENTIRE search space for argos to look for libraries (including
   its own core libraries).

2. If you have installed ARGoS to a non-system path (i.e. something other than
   `/usr/local` or `/usr`), you will also need to update *system* dynamic
   library search paths so the OS can find the libraries that the ARGoS
   executable requires. If you installed it to a system path, then you can skip
   this step. On bash:

        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/local/lib/argos3

   Assuming you have installed ARGoS to `/opt/local`. If you installed it
   somewhere else, then update the path above accordingly.

3. Unless you compile out event reporting (built fordyca with optimizations
   *AND* with `LIBRA_ER_NREPORT=YES` passed to cmake), you will need to set the
   path to the log4cxx configuration file, which tells fordyca which classes
   should have logging turned on, and how verbose to be. On bash that is:

        export LOG4CXX_CONFIGURATION=$HOME/git/fordyca/log4cxx.xml

   Assuming you have cloned and built fordyca in `$HOME/git`. If you cloned and
   built it somewhere else, then update the above path accordingly.

4. cd to the ROOT of the fordyca repo, and run the demo experiment:

        argos3 -c exp/demo.argos

   This should pop up a nice GUI from which you can start the experiment (it
   runs depth0 dpo foraging by default). If no GUI appears, verify that the
   `<visualization>` subtree of the file is not commented out.

# Running on MSI

Head over to
[sierra](https://github.com/swarm-robotics/sierra/tree/devel/README.md), and
follow the MSI setup instructions over there. Don't try to run on MSI without
it. Just don't.

*IMPORTANT* Do not try to run sierra with a debug build of fordyca. It probably
won't work and will be obnoxiously/irritatingly slow if it does.

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
