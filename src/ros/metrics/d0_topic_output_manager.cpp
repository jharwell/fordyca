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
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, metrics, d0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d0_robot_metrics_manager::d0_robot_metrics_manager(
    const rmconfig::metrics_config* const mconfig)
    : crmetrics::robot_metrics_manager(mconfig),
      ER_CLIENT_INIT("fordyca.ros.metrics.d0.d0_manager") {

  /* register collectors common to all of FORDYCA */
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(fmetrics::blocks::manipulation_metrics_collector),
      cmspecs::blocks::kManipulation.xml,
      cmspecs::blocks::kManipulation.scoped,
      rmetrics::output_mode::ekAPPEND }
  };

  rmetrics::register_with_sink<frmetrics::d0::d0_robot_metrics_manager,
                               rmetrics::file_sink_registerer> csv(this,
                                                                   creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmconfig::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);

  boost::mpl::for_each<detail::sink_list>(registerer);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void d0_robot_metrics_manager::collect_from_controller(
    const controller::reactive::d0::crw_controller* const c);

NS_END(d0, metrics, ros, fordyca);
