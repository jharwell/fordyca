/**
 * @file stateless_metrics_aggregator.cpp
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
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/metrics/fsm/distance_metrics.hpp"
#include "fordyca/metrics/fsm/distance_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateless_metrics_aggregator::stateless_metrics_aggregator(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::metrics_params* params,
    const std::string& output_root)
    : base_metrics_aggregator(server, params, output_root) {
  insmod("metrics_agg", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);

  register_collector<metrics::fsm::distance_metrics_collector>(
      "fsm::distance",
      metrics_path() + "/" + params->distance_fname,
      params->collect_interval);

  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "blocks::acquisition",
      metrics_path() + "/" + params->block_acquisition_fname,
      params->collect_interval);

  register_collector<metrics::blocks::transport_metrics_collector>(
      "blocks::transport",
      metrics_path() + "/" + params->block_transport_fname,
      params->collect_interval);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_metrics_aggregator::collect_from_controller(
    const rcppsw::metrics::base_metrics* const controller) {
  auto distance_m =
      dynamic_cast<const metrics::fsm::distance_metrics*>(controller);
  auto block_acq_m =
      dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(controller);

  ER_ASSERT(distance_m,
            "FATAL: Controller does not provide FSM distance metrics");
  ER_ASSERT(block_acq_m,
            "FATAL: Controller does not provide FSM block acquisition metrics");

  collect("fsm::distance", *distance_m);
  collect("blocks::acquisition", *block_acq_m);
} /* collect_from_controller() */

void stateless_metrics_aggregator::collect_from_block(
    const representation::block* const block) {
  collect("blocks::transport", *block);
} /* collect_from_block() */

NS_END(depth0, support, fordyca);
