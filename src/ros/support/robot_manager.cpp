/**
 * \file robot_manager.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ros/support/robot_manager.hpp"

#include "cosm/pal/config/output_config.hpp"
#include "cosm/ros/config/sierra_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
robot_manager::robot_manager(const cros::config::sierra_config* config)
    : ER_CLIENT_INIT("fordyca.ros.support.robot_manager"), mc_sierra(*config) {}

robot_manager::~robot_manager(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void robot_manager::init(ticpp::Element& node) {
  robot_manager_adaptor::init(node);

  /* parse simulation input file */
  config_parse(node);

  /* initialize RNG */
  rng_init(config()->config_get<rmath::config::rng_config>());

  /* initialize output and metrics collection */
  output_init(m_config.config_get<cpconfig::output_config>());
} /* init() */

void robot_manager::config_parse(ticpp::Element& node) {
  m_config.parse_all(node);

  if (!m_config.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }
} /* config_parse() */

/*******************************************************************************
 * ROS Hooks
 ******************************************************************************/
void robot_manager::pre_step(void) {
  /* update current tick */
  timestep(timestep() + 1);

  /* Update diagnostics with new timestep */
  mdc_ts_update();
} /* pre_step() */

bool robot_manager::experiment_finished(void) const {
  return timestep() >=
         mc_sierra.experiment.length.v() * mc_sierra.experiment.ticks_per_sec.v();
} /* experiment_finished() */

NS_END(support, ros, fordyca);
