/**
 * @file base_controller_repository.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/config/base_controller_repository.hpp"
#include "fordyca/config/actuation_parser.hpp"
#include "fordyca/config/exploration_parser.hpp"
#include "fordyca/config/output_parser.hpp"
#include "fordyca/config/sensing_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller_repository::base_controller_repository(void) {
  parser_register<output_parser, output_config>(output_parser::kXMLRoot);
  parser_register<actuation_parser, actuation_config>(
      actuation_parser::kXMLRoot);
  parser_register<sensing_parser, sensing_config>(sensing_parser::kXMLRoot);
  parser_register<exploration_parser, exploration_config>(
      exploration_parser::kXMLRoot);
}

NS_END(config, fordyca);
