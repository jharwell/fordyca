Local Runtime Setup
===================

If you have not successfully completed :ref:`\ln-compilation` part of the setup,
do that first. These steps will not work otherwise.

After successful compilation, follow these steps to run a basic foraging
scenario on your local laptop.

#. Set the ``ARGOS_PLUGIN_PATH`` variable to contain (1) the path to the directory
   containing the ``libfordyca.so`` file, (2) the path to the ARGoS libraries. On
   bash, that is::

     export ARGOS_PLUGIN_PATH=/usr/local/lib/argos3/lib:$HOME/git/fordyca/build/lib

   Assuming you have installed ARGoS to ``/usr/local`` and have cloned/built
   FORDYCA under ``$HOME/git``. If your paths are different, modify the above paths
   accordingly. Note that you need BOTH of these terms in the path, because this
   defines the ENTIRE search space for argos to look for libraries (including
   its own core libraries).

#. If you have installed ARGoS to a non-system path (i.e. something other than
   ``/usr/local`` or ``/usr``), you will also need to update *system* dynamic
   library search paths so the OS can find the libraries that the ARGoS
   executable requires. If you installed it to a system path, then you can skip
   this step. On bash::

     export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/local/lib/argos3

   Assuming you have installed ARGoS to ``/opt/local``. If you installed it
   somewhere else, then update the path above accordingly.

#. Unless you compile out event reporting (built FORDYCA with optimizations
   *AND* with ``LIBRA_ER=LIBRA_ER_NONE`` passed to cmake), you will need to set
   the path to the log4cxx configuration file, which tells FORDYCA which classes
   should have logging turned on, and how verbose to be. On bash that is::

     export LOG4CXX_CONFIGURATION=$HOME/git/fordyca/log4cxx.xml

   Assuming you have cloned and built FORDYCA in ``$HOME/git``. If you cloned and
   built it somewhere else, then update the above path accordingly.

#. cd to the ROOT of the FORDYCA repo, and run the demo experiment::

     argos3 -c exp/demo.argos

   This should pop up a nice GUI from which you can start the experiment (it
   runs depth0 dpo foraging by default). If no GUI appears, verify that the
   ``<visualization>`` subtree of the file is not commented out.

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
     match, as specified :ref:`here ln-xml-config` are correct.
