/**
 * @file depth0_metrics_aggregator.cpp
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
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/config/metrics_config.hpp"
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/metrics/collector_registerer.hpp"
#include "fordyca/metrics/perception/dpo_perception_metrics.hpp"
#include "fordyca/metrics/perception/dpo_perception_metrics_collector.hpp"
#include "fordyca/metrics/perception/mdpo_perception_metrics.hpp"
#include "fordyca/metrics/perception/mdpo_perception_metrics_collector.hpp"
#include "fordyca/repr/base_block.hpp"

#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/movement_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0, detail);

using collector_typelist =
    rmpl::typelist<metrics::collector_registerer::type_wrap<
                       metrics::perception::mdpo_perception_metrics_collector>,
                   metrics::collector_registerer::type_wrap<
                       metrics::perception::dpo_perception_metrics_collector> >;

NS_END(detail);

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::crw_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::dpo_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::mdpo_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::odpo_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::omdpo_controller* const c);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth0_metrics_aggregator::depth0_metrics_aggregator(
    const config::metrics_config* const mconfig,
    const std::string& output_root)
    : base_metrics_aggregator(mconfig, output_root),
      ER_CLIENT_INIT("fordyca.support.depth0.depth0_aggregator") {
  metrics::collector_registerer::creatable_set creatable_set = {
      {typeid(metrics::perception::mdpo_perception_metrics_collector),
       "perception_mdpo",
       "perception:mdpo"},
      {typeid(metrics::perception::dpo_perception_metrics_collector),
       "perception_dpo",
       "perception::dpo"}};

  metrics::collector_registerer registerer(mconfig, creatable_set, this);
  boost::mpl::for_each<detail::collector_typelist>(registerer);

  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class T>
void depth0_metrics_aggregator::collect_from_controller(
    const T* const controller) {
  base_metrics_aggregator::collect_from_controller(controller);

  /*
   * All depth0 controllers provide these.
   */
  collect("fsm::movement", *controller);
  collect("fsm::collision_counts", *controller->fsm());
  collect("blocks::acq_counts", *controller);
  collect("blocks::manipulation", *controller->block_manip_collator());

  collect_if("fsm::collision_locs",
             *controller->fsm(),
             [&](const rmetrics::base_metrics&) {
               return controller->fsm()->ca_tracker()->in_collision_avoidance();
             });

  collect_if("blocks::acq_locs",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               auto& m =
                   dynamic_cast<const cfmetrics::goal_acq_metrics&>(metrics);
               return fsm::foraging_acq_goal::type::ekBLOCK ==
                          m.acquisition_goal() &&
                      m.goal_acquired();
             });

  /*
   * We count "false" explorations as part of gathering metrics on where robots
   * explore.
   */
  collect_if("blocks::acq_explore_locs",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               auto& m =
                   dynamic_cast<const cfmetrics::goal_acq_metrics&>(metrics);
               return m.is_exploring_for_goal().first;
             });
  collect_if("blocks::acq_vector_locs",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               auto& m =
                   dynamic_cast<const cfmetrics::goal_acq_metrics&>(metrics);
               return m.is_vectoring_to_goal();
             });
  /*
   * Only controllers with MDPO perception provide these.
   */
  auto mdpo = dynamic_cast<const metrics::perception::mdpo_perception_metrics*>(
      controller->perception());
  if (nullptr != mdpo) {
    collect("perception::mdpo", *mdpo);
  }
  /*
   * Only controllers with DPO perception provide these.
   */
  auto dpo = dynamic_cast<const metrics::perception::dpo_perception_metrics*>(
      controller->perception());
  if (nullptr != dpo) {
    collect("perception::dpo", *dpo);
  }
} /* collect_from_controller() */

NS_END(depth0, support, fordyca);
