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
#include "fordyca/config/foraging_controller_repository.hpp"

#include "rcppsw/math/config/xml/rng_parser.hpp"
#include "cosm/pal/config/xml/output_parser.hpp"

#include "cosm/repr/config/xml/nest_parser.hpp"
#include "cosm/subsystem/config/xml/actuation_subsystem2D_parser.hpp"
#include "cosm/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"

#include "fordyca/config/strategy/strategy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);
namespace csconfig = csubsystem::config;
namespace cscxml = csconfig::xml;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller_repository::foraging_controller_repository(void) {
  parser_register<cpcxml::output_parser, cpconfig::output_config>(
      cpcxml::output_parser::kXMLRoot);
  parser_register<cscxml::actuation_subsystem2D_parser,
                  csconfig::actuation_subsystem2D_config>(
      cscxml::actuation_subsystem2D_parser::kXMLRoot);
  parser_register<cscxml::sensing_subsystemQ3D_parser,
                  csconfig::sensing_subsystemQ3D_config>(
      cscxml::sensing_subsystemQ3D_parser::kXMLRoot);
  parser_register<fcstrategy::strategy_parser, fcstrategy::strategy_config>(
      fcstrategy::strategy_parser::kXMLRoot);
  parser_register<rmath::config::xml::rng_parser, rmath::config::rng_config>(
      rmath::config::xml::rng_parser::kXMLRoot);
  parser_register<crepr::config::xml::nest_parser, crepr::config::nest_config>(
      crepr::config::xml::nest_parser::kXMLRoot);

  parser_find<cscxml::sensing_subsystemQ3D_parser>(
      cscxml::sensing_subsystemQ3D_parser::kXMLRoot)
      ->ground_detection_add("nest");
  parser_find<cscxml::sensing_subsystemQ3D_parser>(
      cscxml::sensing_subsystemQ3D_parser::kXMLRoot)
      ->ground_detection_add("block");
}

NS_END(config, fordyca);
