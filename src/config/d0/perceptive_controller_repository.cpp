/**
 * \file perceptive_controller_repository.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "fordyca/config/perceptive_controller_repository.hpp"

#include "cosm/subsystem/perception/config/xml/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceptive_controller_repository::perceptive_controller_repository(void) {
  parser_register<cspconfig::xml::perception_parser, cspconfig::perception_config>(
      cspconfig::xml::perception_parser::kXMLRoot);
}

NS_END(config, fordyca);
