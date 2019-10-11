Parser Tutorial
===============

After you have added some new code to FORDYCA, you will (probably) need to be
able to configure your new module(s) from the input ``.argos`` file. To do that
you need to define a new `XML parser`.

This is a tutorial for:

- Adding parameters to the input file
- Adding a parameter struct to the code for said input parameters
- Defining a parser for said input parameters
- Registering said parser so that it is called during initialization.

It assumes that you have already built the documentation for forydca and rcppsw
and have some level of familiarity with the XML parameter parsing.

Adding Parameters To Input File
-------------------------------

#. Identify what parameters you want to add (i.e. what new knobs you want to be
   able to fiddle with).

#. Decide what type of parameter each of the new ones you want to add is: robot
   or simulation. Each type has a different section of the input file that they
   have to go into.

   Robot parameters are things that robots need to do whatever you've told them
   to do. Simulation parameters are for things that are not specific to a single
   robot. Robot parameters are found under the robot controller section, and
   simulation parameters under the loop function section (see
   ``exp/testing.argos`` for good examples of existing parameters).

   All your new parameters may be only in one category, and that's fine.

#. Find an appopriate place in the input file to place said parameters. You
   should add at most 1 XML tag to the robot/simulation parameter section of the
   input file (i.e. all your new parameters for each category should be able to
   be grouped logically/hierarcically under a single XML tag). If you are not
   sure where is appropriate, ask.

   Pick a GOOD name for the root XML tag you add, as that is very important for
   the next step.

Adding ``_config`` Struct
--------------------------

At an appropriate location in the ``fordyca::config`` hierarchy, create a new
configuration struct in a ``.hpp`` file. There are literally dozens of examples
to look at already present, many of which are very simple.

- The parameter struct should be named ``<XML tag>_config`` to help with
  readability and the principle of least surprise. For example, if your tag name
  is ``energy_consumption``, then your parameter struct would be
  ``energy_consumption_config``.

- Each element of the parameter struct should have the SAME name as one of the
  XML attributes under the root tag in the input file, to help with readability
  and the principle of least surprise. For example, if you have an attribute
  called ``rate`` under ``<energy_consumption``, then in
  ``energy_consumption_config`` you would also have a member called ``rate``, of
  whatever type is needed.

Defining an XML Parser
----------------------

At an appropriate location in the ``fordyca::config`` hierarchy, create a new
parser in ``.hpp/.cpp`` files. There are literally dozens of examples to look at
already present, many of which are very simple. The parser should be named
``<XML tag>_parser`` to help with readability and the principle of least
surprise. For example, if your tag name is ``energy_consumption``, then your
parser name would be ``energy_consumption_parser``.

When creating the corresponding ``.cpp`` file, you have two functions you can
override:

- ``parse()``: Does the actual parsing. You MUST used the ``XML_PARSE_PARAM()``
  macro to do most parsing, so that if you do not name struct members with the
  same name as the input file attribute, you will get compile time rather than
  run time errors.

- ``validate()``: Does any validation of parsed parameters. Optional. Mainly used
  to make sure things like angles are always > 0 but < 360, for example.


Registering a New Parser
------------------------

Depending on what controller/loop functions are going to need your parameters,
you will need to register your parser the corresponding parameter
repository. For example, if I create an ``energy_consumption_parser`` for use by
depth0 controllers, I would register my parser with the
``depth0_controller_repository`` (see ``.cpp`` file for examples of how--it is
quite straightforward.).

That's it!
