.. _ln-build:

Building The Code Locally
=========================

.. IMPORTANT:: If you want to build an optimized version of FORDYCA (necessary
   for large swarms), make sure you pass ``--opt`` to the ``bootstrap.sh``
   script that you copied, or re-run ``cmake`` and ``make`` as shown in the
   :ref:`ln-opt-build` section below, as that script is for a debug build by
   default.


Debug Build
-----------

.. IMPORTANT:: These instructions assume:

   - You are running on a Debian-based linux environment, specifically
     Ubuntu. If you are running on something else (OSX, WSL, etc) you will have
     to manually modify the script to work on your target platform and/or
     probably have to build a **LOT** more stuff from source manually.

   - You have sudo privileges on the machine you want to install the project on.

   If either of these conditions are not met, you will be on your own for
   getting things setup in your development environment of choice.

Download ``scripts/bootstrap.sh`` BEFORE cloning the FORDYCA repo. The script
can be downloaded by navigating to the file on github, clicking the ``raw``
button, and then right clicking and doing ``Save As``. After downloading, mark
the script as executable (``chmod +x bootstrap.sh``) and then run it (it can be
run from anywhere), with the following arguments:

.. IMPORTANT:: Make sure you download the ``bootstrap.sh`` script from the
               ``devel`` branch of the FORDYCA repo. The one on the ``master``
               branch may be out of date.

The bootstrap script takes a number of arguments, which you can read about via::

  ./bootstrap.sh --help

The default values of the arguments should be OK for most use cases. Some
important considerations:

- If you set ``--prefix`` to a system location, then it is assumed you have sudo
  access.

- If set ``--prefix`` to a system location, then you will not need to set
  ``LD_LIBRARY_PATH`` later so bash can find it.

For example::

  ./bootstrap.sh  > output.txt 2>&1

Will install system packages, build the code under ``$HOME/research`` and
install ARGoS to ``$HOME/.local``. The ``> output.txt 2>&1`` part is
important to capture the output of running the script so that if there are
errors it is easier to track them down (the script generates a LOT of output,
which usually overflows terminal ringbuffers).

The script is configured such that it will stop if any command fails. So if the
script makes it all the way through and you see a ``BOOTSTRAP SUCCESS!`` line at
the end of the ``output.txt``, then you know everything worked. Otherwise look
in the ``output.txt`` for the error and fix it and try running the script again
(the script **should** be idempotent).

 .. _ln-opt-build:

Optimized Build
---------------

To build ``FORDYCA`` with optimizations (necessary for using ``SIERRA`` or
running large scale simulations), will need a different cmake command than the
one ``bootstrap.sh`` uses for you. Something like the following, run from the
``build`` directory prior to building will do the trick::

  cmake -DCMAKE_C_COMPILER=gcc-9 \
  -DCMAKE_CXX_COMPILER=g++-9 \
  -DCMAKE_BUILD_TYPE=OPT \
  -DLIBRA_ER=NONE \
  -DWITH_FOOTBOT_BATTERY=NO \
  -DWITH_FOOTBOT_RAB=NO \
  -DWITH_FOOTBOT_LEDS=NO \
  -DWITH_FOOTBOT_CAMERA=NO \
  \..

To get an idea of what some of the non-project specific options mean, head over
to the :xref:`LIBRA` docs.

``WITH_FOOTBOT_BATTERY``, ``WITH_FOOTBOT_RAB``, ``WITH_FOOTBOT_LEDS``,
``WITH_FOOTBOT_CAMERA`` are things that are only needed if you are running
experiments which utilize those sensors/actuators, otherwise they slow things
down a **LOT** with large swarms (which is why you are compiling with
optimizations on in the first place).

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
