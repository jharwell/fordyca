/**
 * \file argos_swarm_manager_repository.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/argos/support/config/argos_swarm_manager_repository.hpp"

#include "fordyca/argos/support/caches/config/caches_parser.hpp"
#include "fordyca/argos/support/tv/config/tv_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
argos_swarm_manager_repository::argos_swarm_manager_repository(void) noexcept {
  parser_register<fastv::config::tv_manager_parser,
                  fastv::config::tv_manager_config>(
                      fastv::config::tv_manager_parser::kXMLRoot);
  parser_register<fascaches::config::caches_parser,
                  fascaches::config::caches_config>(
                      fascaches::config::caches_parser::kXMLRoot);
}

NS_END(config, support, argos, fordyca);
