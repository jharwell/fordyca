/**
 * \file d0_robot_metrics_manager.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ros/metrics/d0/d0_robot_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/network_sink_registerer.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/metrics/specs.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ros/metrics/topic_sink.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/specs.hpp"
#include "fordyca/ros/metrics/blocks/manipulation_metrics_topic_sink.hpp"
#include "fordyca/ros/metrics/registrable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, metrics, d0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d0_robot_metrics_manager::d0_robot_metrics_manager(
    const cros::topic& robot_ns,
    const rmconfig::metrics_config* const mconfig)
    : crmetrics::robot_metrics_manager(robot_ns, mconfig),
      ER_CLIENT_INIT("fordyca.ros.metrics.d0.d0_robot_metrics_manager") {
  using sink_list = rmpl::typelist<
      rmpl::identity<frmetrics::blocks::manipulation_metrics_topic_sink> >;

  ER_INFO("Registering collectors");

  rmetrics::register_with_sink<frmetrics::d0::d0_robot_metrics_manager,
                               rmetrics::network_sink_registerer>
      topic(this, frmetrics::registrable::kStandard);
  rmetrics::register_using_config<decltype(topic), rmconfig::network_sink_config>
      registerer(std::move(topic), &mconfig->network);

  boost::mpl::for_each<sink_list>(registerer);

  /* setup metric collection for all collector groups in all sink groups */
  ER_INFO("Initializing collectors");
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class TController>
void d0_robot_metrics_manager::collect_from_controller(
    const TController* const c) {
  crmetrics::robot_metrics_manager::collect_from_controller(c);

  /*
   * All d0 controllers provide these.
   */
  collect(cmspecs::spatial::kNestZone.scoped(), *c->nz_tracker());
  collect(cmspecs::strategy::nest::kAcq.scoped(), *c->fsm());
  collect(cmspecs::blocks::kAcqCounts.scoped(), *c);
  collect(cmspecs::blocks::kTransporter.scoped(), *c);
  collect(fmspecs::blocks::kManipulation.scoped(), *c->block_manip_recorder());
} /* collect_from_controller() */


/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void d0_robot_metrics_manager::collect_from_controller(
    const fcrd0::crw_controller* const c);

NS_END(d0, metrics, ros, fordyca);
