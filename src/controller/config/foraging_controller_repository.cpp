/**
 * \file foraging_controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/foraging_controller_repository.hpp"

#include "cosm/hal/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"
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

  parser_find<chsubsystem::config::xml::sensing_subsystemQ3D_parser>(
      chsubsystem::config::xml::sensing_subsystemQ3D_parser::kXMLRoot)
      ->env_detection_add("nest");
  parser_find<chsubsystem::config::xml::sensing_subsystemQ3D_parser>(
      chsubsystem::config::xml::sensing_subsystemQ3D_parser::kXMLRoot)
      ->env_detection_add("block");
}

NS_END(config, controller, fordyca);
