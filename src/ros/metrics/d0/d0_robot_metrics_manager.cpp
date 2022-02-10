/**
 * \file d0_robot_metrics_manager.cpp
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
#include "fordyca/ros/metrics/d0/d0_robot_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/network_sink_registerer.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/ros/metrics/topic_sink.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/metrics/specs.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
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
    rmpl::identity<frmetrics::blocks::manipulation_metrics_topic_sink>
    >;

  ER_INFO("Registering collectors");

  rmetrics::register_with_sink<frmetrics::d0::d0_robot_metrics_manager,
                               rmetrics::network_sink_registerer> topic(this,
                                                                        frmetrics::registrable::kStandard);
  rmetrics::register_using_config<decltype(topic),
                                  rmconfig::network_sink_config> registerer(
                                      std::move(topic),
                                      &mconfig->network);

  boost::mpl::for_each<sink_list>(registerer);

  /* setup metric collection for all collector groups in all sink groups */
  ER_INFO("Initializing collectors");
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void d0_robot_metrics_manager::collect_from_controller(
    const fcontroller::foraging_controller* const c) {
  crmetrics::robot_metrics_manager::collect_from_controller(c);

  /*
   * All d0 controllers provide these.
   */
  collect(fmspecs::blocks::kManipulation.scoped, *c->block_manip_recorder());
} /* collect_from_controller() */

NS_END(d0, metrics, ros, fordyca);
