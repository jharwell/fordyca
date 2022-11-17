/**
 * \file d0_robot_metrics_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
