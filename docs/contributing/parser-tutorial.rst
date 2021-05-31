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

   For example, you might have:

   .. code-block:: XML

      <widget>
         <subwidget
            param1="10"
            param2="FOOBAR"/>
      </widget>


Adding ``_config`` Struct
--------------------------

At an appropriate location in the ``fordyca::config`` hierarchy, create a new
configuration struct in a ``.hpp`` file. There are literally dozens of examples
to look at already present in the code, many of which are very simple.

- The parameter struct should be named ``<XML tag>_config`` to help with
  readability and the principle of least surprise. For example, if your XML tag
  name is ``subwidget``, then your parameter struct would be
  ``subwidget_config``. This is the Principle of Least Surprise at
  work.

- Each element of the parameter struct should have the SAME name as one of the
  XML attributes under the root tag in the input file, to help with readability
  and the principle of least surprise. For example, if you have an attribute
  called ``rate`` under ``<energy_consumption``, then in
  ``energy_consumption_config`` you would also have a member called ``rate``, of
  whatever type is needed. This is enforced during parsing in the C++ code.

 For our ``<subwidget>`` example, we would defined:

  .. code-block:: cpp

     struct subwidget_config {
       int param1;
       std::string param2;
     }

Defining an XML Parser
----------------------

At an appropriate location in the ``fordyca::config`` hierarchy, create a new
parser in ``.hpp/.cpp`` files. There are literally dozens of examples to look at
already present, many of which are very simple. The parser should be named
``<XML tag>_parser`` to help with readability and the Principle of Least
Surprise. For example, if your tag name is ``subwidget``, then your parser name
would be ``subwidget_parser``.

When creating your class, you `must` define the following inside the ``public``
access modifier:

- ``config_type`` - Set to the type of the config struct for your class. This
  is a convention/requirement that allows lookup of the specific type of config
  a parser is parsing without casing on the parser name. Because of the above
  conventions, this should be the name of your parser class, changing
  ``_parser`` to ``_config``.

When creating your class, you `must` override:

- `config_get_impl()` - This returns a non-owning reference to the internal
  parsed config struct. If the config struct has not been populated (e.g., the
  necessary tag for the parser to parse was not in the input .argos file), then
  it should return ``nullptr``.

- ``xml_root()`` - Return the name of the XML tree that your parser is
  parsing. Because of the conventions used, this should be the name of your
  parser class, minus the ``_parser`` at the end.

- ``parse()``: Does the actual parsing. The function is passed the
  ``ticpp::Element`` node of the `parent` XML tag that your parser is parsing;
  this convention allows easy parser nesting and clean parsing regardless if an
  XML tag is expected to exist or not.

  For example, if you are defining a ``subwidget_parser`` class, you might have
  the following XML structure:

  .. code-block:: XML

     <widget>
        <subwidget
           param1="10"
           param2="FOOBAR"/>
     </widget>


  With such an XML structure, your ``subwidget_parser::parser()`` function will
  be passed a reference to the ``<widget>`` tree, and you will need to call
  ``node_get("subwidget")`` to get a reference to the XML tree rooted at
  ``<subwidget>``.

  You MUST use the ``XML_PARSE_ATTR()`` macro to do most parsing, so that if you
  do not name struct members with the same name as the input file attribute, you
  will get compile time rather than run time errors. If you want to have an
  optional attribute, you can supply a default via
  ``XML_PARSE_ATTR_DFLT()``. Otherwise, a missing attribute will cause a
  run-time error.

  A possible implementation for the ``.hpp`` file might be:

  .. code-block:: cpp

     class subwidget_parser final : public rconfig::xml::xml_config_parser {
      public:
       using config_type = subwidget_config;

       /**
        * \brief The root tag that all XML configuration for exploration should lie
        * under in the XML tree.
        */
       inline static const std::string kXMLRoot = "subwidget";

       void parse(const ticpp::Element& node) override;
       std::string xml_root(void) const override { return kXMLRoot; }

      private:
       const rconfig::base_config* config_get_impl(void) const override {
         return m_config.get();
       }

       /* clang-format off */
       std::unique_ptr<config_type> m_config{nullptr};
       /* clang-format on */
     };

  A possible implementation for the ``.cpp`` file might be:

  .. code-block:: cpp

     void subwidget_parser::parse(const ticpp::Element& node) {
       /* If our subtree not in input file, nothing to do */
       if (nullptr == node.FirstChild(xml_root(), false)) {
         return;
       }
       ticpp::Element vnode = node_get(node, xml_root());
       m_config = std::make_unique<config_type>();

       XML_PARSE_ATTR(vnode, m_config, param1);
       XML_PARSE_ATTR_DFLT(vnode, m_config, param2, std::string());
     } /* parse() */


When creating your class you `can` override:

- ``validate()``: Does any validation of parsed parameters. Mainly used to make
  sure things like angles are always > 0 but < 360, for example, which is not
  applicable to all parsers.


Registering a New Parser
------------------------

Depending on what controller/loop functions are going to need your parameters,
you will need to register your parser the corresponding parameter
repository. For example, if I create an ``energy_consumption_parser`` for use by
d0 controllers, I would register my parser with the
``d0_controller_repository`` via something like:

.. code-block:: cpp

   parser_register<energy_consumption_praser,
                   energy_consumption_config>(
   energy_consumption_praser::kXMLRoot);

That's it!
