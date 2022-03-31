/**
 * \file d0_swarm_metrics_manager.cpp
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
#include "fordyca/ros/metrics/d0/d0_swarm_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/pal/ros/swarm_iterator.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/metrics/specs.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_csv_sink.hpp"
#include "fordyca/ros/metrics/registrable.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/ros/metrics/blocks/manipulation_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, metrics, d0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d0_swarm_metrics_manager::d0_swarm_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const fs::path& root,
    size_t n_robots)
    : crmetrics::swarm_metrics_manager(mconfig, root, n_robots),
      ER_CLIENT_INIT("fordyca.ros.metrics.d0.d0_swarm_metrics_manager") {

  /*
   * Register all standard metrics which don't require additional parameters
   * and can be done by default.
   */
  register_standard(mconfig, n_robots);

  std::all_of(std::begin(m_subs),
              std::end(m_subs),
              [&](auto& sub) {
                return wait_for_connection(sub);
              });

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void d0_swarm_metrics_manager::register_standard(
    const rmconfig::metrics_config* const mconfig,
    size_t n_robots) {
  using sink_list = rmpl::typelist<
    rmpl::identity<fmetrics::blocks::manipulation_metrics_csv_sink>
    >;

  /* register collectors common to all of FORDYCA */
  rmetrics::register_with_sink<frmetrics::d0::d0_swarm_metrics_manager,
                               rmetrics::file_sink_registerer> topic(this,
                                                                     registrable::kStandard);
  rmetrics::register_using_config<decltype(topic),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(topic),
                                      &mconfig->csv);

  boost::mpl::for_each<sink_list>(registerer);

  /* initialize counting map to track received metrics */
  msg_tracking()->init(fmspecs::blocks::kManipulation.scoped());

  /* set ROS callbacks for metric collection */
  ::ros::NodeHandle n;
  auto cb = [&](cros::topic robot_ns) {
              m_subs.push_back(n.subscribe<frmblocks::manipulation_metrics_msg>(
                  robot_ns / fmspecs::blocks::kManipulation.scoped(),
                  kQueueBufferSize,
                  &d0_swarm_metrics_manager::collect,
                  this));
            };
  cpros::swarm_iterator::robots(n_robots, cb);
} /* register_standard() */

/*******************************************************************************
 * ROS Callbacks
 ******************************************************************************/
void d0_swarm_metrics_manager::collect(
    const boost::shared_ptr<const frmblocks::manipulation_metrics_msg>& msg) {
  auto* collector = get<fmetrics::blocks::manipulation_metrics_collector>(
      fmspecs::blocks::kManipulation.scoped());
  msg_tracking()->update_on_receive(fmspecs::blocks::kManipulation.scoped(),
                                    msg->header.seq);
  collector->collect(msg->data);
} /* collect() */

NS_END(d0, metrics, ros, fordyca);
