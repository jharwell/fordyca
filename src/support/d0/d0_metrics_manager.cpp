/**
 * \file d0_metrics_manager.cpp
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
#include "fordyca/support/d0/d0_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

#include "rcppsw/metrics/collector_registerer.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/ds/cell2D.hpp"

#include "fordyca//controller/foraging_controller.hpp"
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/metrics/perception/dpo_metrics.hpp"
#include "fordyca/metrics/perception/dpo_metrics_collector.hpp"
#include "fordyca/metrics/perception/dpo_metrics_csv_sink.hpp"
#include "fordyca/metrics/perception/mdpo_metrics.hpp"
#include "fordyca/metrics/perception/mdpo_metrics_collector.hpp"
#include "fordyca/metrics/perception/mdpo_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d0, detail);

using sink_list = rmpl::typelist<
    rmpl::identity<metrics::perception::mdpo_metrics_csv_sink>,
    rmpl::identity<metrics::perception::dpo_metrics_csv_sink> >;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d0_metrics_manager::d0_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const fs::path& output_root,
    size_t n_block_clusters)
    : fordyca_metrics_manager(mconfig, gconfig, output_root, n_block_clusters),
      ER_CLIENT_INIT("fordyca.support.d0.d0_manager") {
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(metrics::perception::mdpo_metrics_collector),
      "perception_mdpo",
      "perception::mdpo",
      rmetrics::output_mode::ekAPPEND },
    { typeid(metrics::perception::dpo_metrics_collector),
      "perception_dpo",
      "perception::dpo",
      rmetrics::output_mode::ekAPPEND }
  };

  rmetrics::register_with_csv_sink csv(&mconfig->csv,
                                       creatable_set,
                                       this);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<detail::sink_list>(registerer);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class TController>
void d0_metrics_manager::collect_from_controller(const TController* const controller) {
  fordyca_metrics_manager::collect_from_controller(controller);

  /*
   * All d0 controllers provide these.
   */
  collect("spatial::movement", *controller);
  collect("strategy::nest_acq", *controller->fsm());
  collect("fsm::interference_counts", *controller->fsm());
  collect("blocks::acq_counts", *controller);
  collect("blocks::transporter", *controller);
  collect("blocks::manipulation", *controller->block_manip_recorder());

  collect_if("fsm::interference_locs2D",
             *controller->fsm(),
             [&](const rmetrics::base_metrics&) {
               return controller->fsm()->inta_tracker()->exp_interference();
             });

  collect_if("blocks::acq_locs2D",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               const auto& m =
                   dynamic_cast<const csmetrics::goal_acq_metrics&>(metrics);
               return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
                   m.goal_acquired();
             });

  /*
   * We count "false" explorations as part of gathering metrics on where robots
   * explore.
   */
  collect_if("blocks::acq_explore_locs2D",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               const auto& m =
                   dynamic_cast<const csmetrics::goal_acq_metrics&>(metrics);
               return m.is_exploring_for_goal().is_exploring;
             });
  collect_if("blocks::acq_vector_locs2D",
             *controller,
             [&](const rmetrics::base_metrics& metrics) {
               const auto& m =
                   dynamic_cast<const csmetrics::goal_acq_metrics&>(metrics);
               return m.is_vectoring_to_goal();
             });
  collect_from_cognitive_controller(controller);
} /* collect_from_controller() */

template<class TController,
         typename U,
         RCPPSW_SFINAE_DEF(std::is_base_of<fccognitive::cognitive_controller,
                           U>::value)>
void d0_metrics_manager::collect_from_cognitive_controller(const TController* controller)  {
  collect("perception::dpo", *controller->perception());
  collect("perception::mdpo", *controller->perception());
} /* collect_from_cognitive_controller() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void d0_metrics_manager::collect_from_controller(
    const controller::reactive::d0::crw_controller* const c);
template void d0_metrics_manager::collect_from_controller(
    const controller::cognitive::d0::dpo_controller* const c);
template void d0_metrics_manager::collect_from_controller(
    const controller::cognitive::d0::mdpo_controller* const c);
template void d0_metrics_manager::collect_from_controller(
    const controller::cognitive::d0::odpo_controller* const c);
template void d0_metrics_manager::collect_from_controller(
    const controller::cognitive::d0::omdpo_controller* const c);

NS_END(d0, support, fordyca);
