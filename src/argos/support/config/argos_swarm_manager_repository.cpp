/**
 * \file argos_swarm_manager_repository.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
