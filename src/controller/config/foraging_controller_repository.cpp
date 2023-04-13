/**
 * \file foraging_controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/foraging_controller_repository.hpp"

#include "cosm/hal/subsystem/config/xml/sensing_subsystem_parser.hpp"
#include "cosm/repr/config/xml/nest_parser.hpp"

#include "fordyca/strategy/config/strategy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller_repository::foraging_controller_repository(void) {
  parser_register<fsconfig::strategy_parser, fsconfig::strategy_config>(
      fsconfig::strategy_parser::kXMLRoot);
  parser_register<crepr::config::xml::nest_parser, crepr::config::nest_config>(
      crepr::config::xml::nest_parser::kXMLRoot);

  parser_find<chsubsystem::config::xml::sensing_subsystem_parser>(
      chsubsystem::config::xml::sensing_subsystem_parser::kXMLRoot)
      ->env_detection_add("nest");
  parser_find<chsubsystem::config::xml::sensing_subsystem_parser>(
      chsubsystem::config::xml::sensing_subsystem_parser::kXMLRoot)
      ->env_detection_add("block");
}

NS_END(config, controller, fordyca);
