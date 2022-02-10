/**
 * \file d0_swarm_manager.cpp
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
#include "fordyca/ros/support/d0/d0_swarm_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/pal/ros/swarm_iterator.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/ros/metrics/d0/d0_swarm_metrics_manager.hpp"
#include "fordyca/ros/support/d0/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, support, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
d0_swarm_manager::d0_swarm_manager(const cros::config::sierra_config* config)
    : ER_CLIENT_INIT("fordyca.ros.support.d0.d0_swarm_manager"),
      swarm_manager(config),
      m_metrics_manager(nullptr) {}

d0_swarm_manager::~d0_swarm_manager(void) = default;

/*******************************************************************************
 * Initialization
 ******************************************************************************/
void d0_swarm_manager::init(ticpp::Element& node) {
  mdc_ts_update();
  ndc_uuid_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void d0_swarm_manager::shared_init(ticpp::Element& node) {
  frsupport::swarm_manager::init(node);
} /* shared_init() */

void d0_swarm_manager::private_init(void) {
  /* initialize output and metrics collection */
  const auto* output = config()->config_get<cpconfig::output_config>();
  m_metrics_manager = std::make_unique<frmetrics::d0::d0_swarm_metrics_manager>(
      &output->metrics,
      output_root(),
      swarm_size());
} /* private_init() */

/*******************************************************************************
 * ROS Hooks
 ******************************************************************************/
void d0_swarm_manager::post_step(void) {
  ndc_uuid_push();
  frsupport::swarm_manager::post_step();
  ndc_uuid_pop();

  ndc_uuid_push();

  m_metrics_manager->flush(rmetrics::output_mode::ekTRUNCATE, timestep());
  m_metrics_manager->flush(rmetrics::output_mode::ekCREATE, timestep());
  m_metrics_manager->flush(rmetrics::output_mode::ekAPPEND, timestep());
  ER_DEBUG("Flushed metrics to file");

  m_metrics_manager->interval_reset(timestep());

  ndc_uuid_pop();
} /* post_step() */

void d0_swarm_manager::destroy(void) {
  if (nullptr != m_metrics_manager) {
    m_metrics_manager->finalize();
  }
} /* destroy() */

void d0_swarm_manager::reset(void) {
  ndc_uuid_push();
  frsupport::swarm_manager::reset();
  m_metrics_manager->initialize();
  ndc_uuid_pop();
} /* reset() */

NS_END(d0, support, ros, fordyca);
