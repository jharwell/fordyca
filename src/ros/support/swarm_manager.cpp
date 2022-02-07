/**
 * \file swarm_manager.cpp
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
#include "fordyca/ros/support/swarm_manager.hpp"

#include "cosm/pal/config/output_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_manager::swarm_manager(void)
    : ER_CLIENT_INIT("fordyca.ros.support.swarm_manager") {}

swarm_manager::~swarm_manager(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void swarm_manager::init(ticpp::Element& node) {
  swarm_manager_adaptor::init(node);

  /* parse simulation input file */
  config_parse(node);

  /* initialize RNG */
  rng_init(config()->config_get<rmath::config::rng_config>());

  /* initialize output and metrics collection */
  output_init(m_config.config_get<cpconfig::output_config>());
} /* init() */

void swarm_manager::config_parse(ticpp::Element& node) {
  m_config.parse_all(node);

  if (!m_config.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }
} /* config_parse() */

NS_END(support, ros, fordyca);
