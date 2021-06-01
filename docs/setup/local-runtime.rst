.. _ln-local-runtime:

Local Runtime Setup
===================

If you have not successfully completed :ref:`ln-build` part of the setup, do
that first. These steps will not work otherwise.

After successful compilation, follow these steps to setup the FORDYCA runtime
environment and run a basic foraging scenario on your local laptop.

.. NOTE:: If you don't want to go through this runtime setup each time you start
          a new shell, add whatever commands you run in the terminal to
          ``$HOME/.bashrc`` (or whatever the startup file for your shell is) to
          have them run automatically when you login.

#. Update the *system* dynamic library search paths so the OS can find the
   libraries that the ARGoS executable requires (supposedly ARGoS will do this
   for you when you install it via ldconfig to ``/usr/local``, but many people
   still have trouble with it). On bash::

     export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/lib/argos3

   Assuming you passed ``--prefix=$HOME/.local`` to the ``bootstrap.sh``. If you
   passed something else, then update the path above accordingly.

#. Update your ``PATH`` so that the shell can find ARGoS. On bash::

       export PATH=$PATH:/opt/.local/bin

   Assuming that you passed ``--prefix=$HOME/.local/bin`` to the
   ``bootstrap.sh``. If you passed something else, then update the above path
   accordingly.

   .. NOTE:: If you passed ``--prefix=/usr/local`` to the ``bootstrap.sh``
             script you can skip this step.

#. Set the ``ARGOS_PLUGIN_PATH`` variable to contain (1) the path to the
   directory containing the ``libfordyca.so`` file, (2) the path to the ARGoS
   libraries. On bash, that is::

     export ARGOS_PLUGIN_PATH=/opt/.local/lib/argos3/lib:$HOME/research/fordyca/build/lib

   Assuming that you passed ``--prefix=/opt/.local --rroot=$HOME/research`` to
   the ``bootstrap.sh`` script when you built FORDYCA. If your paths are
   different, modify the above paths accordingly. Note that you need BOTH of
   these terms in the path, because this defines the ENTIRE search space for
   argos to look for libraries (including its own core libraries).

#. Unless you compile out event reporting (built FORDYCA with optimizations
   *AND* with ``LIBRA_ER=NONE`` passed to cmake), you will need to set
   the path to the log4cxx configuration file, which tells FORDYCA which classes
   should have logging turned on, and how verbose to be. On bash that is::

     export LOG4CXX_CONFIGURATION=$HOME/research/fordyca/log4cxx.xml

   Assuming you have cloned and built FORDYCA in ``$HOME/research``. If you
   cloned and built it somewhere else, then update the above path accordingly.

#. cd to the ROOT of the FORDYCA repo, and run the demo experiment::

     argos3 -c exp/demo.argos

   This should pop up a nice GUI from which you can start the experiment (it
   runs depth0 dpo foraging by default). If the simulation seems to start but no
   GUI appears, verify that the ``<visualization>`` subtree of ``demo.argos``
   file is not commented out.

.. IMPORTANT:: You should (probably) have ``n_threads`` set to 0 in the
   ``.argos`` file or omit the attribute altogether when running debug
   builds. When debugging you want things to be executed in a deterministic
   manner, and non-deterministic parallel execution with multiple threads should
   be used only for optimized builds once you are confident of code correctness.

Runtime Issues
--------------

Before reporting a bug, try:

  #. If you are getting a segfault when running ARGoS, verify that if you are
     running with Qt visualizations that the threadcount is 0 (Qt5 cannot run
     with multiple threads without segfaulting).

  #. Verify you don't have any anaconda bits in your ``PATH``. Depending on
     version, anaconda loads a DIFFERENT version of the Qt than fordyca uses,
     resulting in a dynamic linking error.

  #. Make sure you have the necessary environment variables set correctly.

  #. If you get a ``std::bad_cast``, ``boost::get``, or similar exception, then
     verify that the name of [controller, loop functions, qt user functions],
     match, as specified in :ref:`ln-xml-config` are correct.
