/**
 * \file d0_robot_metrics_manager.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/robot_metrics_manager.hpp"

#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, metrics, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d0_robot_metrics_manager
 * \ingroup ros metrics d0
 *
 * \brief Writes metrics gathered from robots to ROS topics to be ent to the ROS
 * master for aggregation and processing. Currently includes:
 *
 * - FSM distance/block acquisition metrics
 */

class d0_robot_metrics_manager : public crmetrics::robot_metrics_manager,
                                 public rer::client<d0_robot_metrics_manager> {
 public:
  d0_robot_metrics_manager(const cros::topic& robot_ns,
                           const rmconfig::metrics_config* mconfig);

  template<class T>
  void collect_from_controller(const T* controller);
};

NS_END(d0, metrics, ros, fordyca);
