.. _ln-build:

Building The Code Locally
=========================

.. IMPORTANT:: If you want to build an optimized version of FORDYCA (necessary
   for large swarms), you will need to either manually modify the
   ``bootstrap.sh`` script that you copied, or re-run ``cmake`` and ``make`` as
   shown in the :ref:`ln-opt-build` section below, as that script is for a debug
   build by default.

Debug Build
-----------

.. IMPORTANT:: These instructions assume:

   - You are running on a Debian-based linux environment, specifically
     Ubuntu. If you are running on something else (OSX, WSL, etc) you will have
     to manually modify the script to work on your target platform and/or
     probably have to build a **LOT** more stuff from source manually.

   - You have sudo privileges on the machine you want to install the project on.

   If either of these conditions is not met, you will be on your own for getting
   things setup in your development environment of choice.

Download ``scripts/bootstrap.sh`` BEFORE cloning the FORDYCA repo. The script
can be downloaded by navigating to the file on github, clicking the ``raw``
button, and then right clicking and doing ``Save As``. After downloading, mark
the script as executable (``chmod +x bootstrap.sh``) and then run it (it can be
run from anywhere), with the following arguments:

- 1st arg: The root directory for the project (all repos will be cloned/built
  in here, and it **must** be an absolute path).
- 2nd arg: ``YES`` if you want to install ARGoS system wide (you probably do)
  and ``NO`` otherwise.
- 3rd arg: Location ARGoS should be installed into.
- 4th arg: The # of cores to use when building ARGoS/FORDYCA (should be set to
  # cores on your machine).

For example::

  ./bootstrap.sh $HOME/research YES /usr/local 2 > output.txt 2>&1

To build the code under ``~/research`` on a 2 core machine and install ARGoS
system-wide. The ``> output.txt 2>&1`` part is important to capture the output
of running the script so that if there are errors it is easier to track them
down (the script generates a LOT of output, which usually overflows terminal
ringbuffers).

The script is configured such that it will stop if any command fails. So if the
script makes it all the way through and you see a ``BOOTSTRAP SUCCESS!`` line at
the end of the ``output.txt``, then you know everything worked. Otherwise look
in the ``output.txt`` for the error and fix it and try running the script again
(the script **should** be idempotent).

 .. _ln-opt-build:

Optimized Build
---------------

To build ``FORYDCA`` with optimizations (necessary for using ``SIERRA`` or
running large scale simulations), will need a different cmake command than the
one ``bootstrap.sh`` uses for you. Something like the following, run from the
``build`` directory prior to building will do the trick::

  cmake -DCMAKE_C_COMPILER=gcc-8 \
  -DCMAKE_CXX_COMPILER=g++-8 \
  -DWITH_FOOTBOT_BATTERY=NO \
  -DWITH_FOOTBOT_RAB=NO \
  -DWITH_FOOTBOT_LEDS=NO \
  -DCMAKE_BUILD_TYPE=OPT \
  -DLIBRA_ER=NONE \
  -DLIBRA_OPENMP=YES \
  -DLIBRA_BUILD_FOR=ARGOS \
  \..

To get an idea of what some of the non-project specific options mean, head over
to the `libra <https://github.com/swarm-robotics/libra/tree/devel/README.md>`_
repo and look at the README.

.. IMPORTANT:: ``LIBRA_OPENMP`` should only be set to ``YES`` if you are going
   to be running with really large swarms (> 1000), otherwise the overhead from
   using threads will probably be greater than any speedup you will get. YMMV.

``WITH_FOOTBOT_BATTERY``, ``WITH_FOOTBOT_RAB``, ``WITH_FOOTBOT_LEDS`` are things
that are only needed if you are running experiments which utilize those
sensors/actuators, otherwise they slow things down a **LOT** with large swarms
(which is why you are compiling with optimizations on in the first place).

Build Notes
-----------

- If you do a build with the intel compiler toolchain, then Qt graphical
  displays are disabled (cmake 3.10 does not correctly handle icpc for C++17).

.. WARNING:: Whatever compiler you use to build FORDYCA should be the same
  compiler that you use to build ARGoS, otherwise ARGoS will be unable to load
  ``libfordyca.so``. Why? Because the ABI compatability documentation between
  icpc and gcc/clang LIES when it promises compability (has to do with ARGoS
  using dlopen() and C++ name mangling I think). ``gcc`` and ``clang``
  interoperability sometimes is OK (i.e. build ARGoS with one, and then build
  FORDYCA with the other), depending on the versions used.

Viewing The Documentation
-------------------------

After the bootstrap.sh script finishes successfully, you can (*AND SHOULD*) view
the doxygen documentation in your browser by navigating to the generated
``index.html`` file. Simply open your browser, and then put the path to the
fordyca repo followed by ``/build/docs/fordyca/html/index.html``. For example,
if you built fordyca under ``$HOME/research``, then you would do
``$HOME/research/fordyca/build/docs/fordyca/html/index.html`` in the address bar
of your browser.

Alternatively, if you would like a .pdf of the documentation, you can navigate
to the ``latex`` directory for doxygen and then build said pdf. Again assuming
you built fordyca in ``$HOME/research``, do the following::

  cd $HOME/research/fordyca/build/docs/fordyca/latex
  make

A ``refman.pdf`` will (eventually) be built in that directory once the command
finishes. Note that if you want to do build the .pdf you will also need the
following programs:

- ``pdflatex`` (``texlive-latex-base`` on ubuntu)
- Texlive fonts (``texlive-fonts-extra`` on ubuntu)


Build Issues
------------

Before reporting a bug, try:

#. Verifying that FORDYCA, COSM, RCPPSW are all on the ``devel`` branch.

#. Updating RCPPSW, COSM, FORDYCA to the latest ``devel`` branch via ``git
   pull``.

#. Updating the FORDYCA, COSM, RCPPSW cmake submodules by running::

     git submodule update --recursive --remote

   in the root of each repository.


If the problem perists, open an issue.
