/**
 * @file collision_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class collision_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics should be collected regarding
 * collision avoidance as robots go about their tasks.
 */
class collision_metrics : public virtual rmetrics::base_metrics {
 public:
  collision_metrics(void) = default;
  ~collision_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then a robot is currently engaged in collision avoidance.
   */
  virtual bool in_collision_avoidance(void) const = 0;

  /**
   * @brief If \c TRUE, then a robot has just entered collision avoidance. This
   * should return \c FALSE on all subsequent steps the robot is in collision
   * avoidance.
   */
  virtual bool entered_collision_avoidance(void) const = 0;

  /**
   * @brief If \c TRUE, then a robot has just exited collision avoidance. This
   * should return \c FALSE on all previous steps the robot is in collision
   * avoidance, and all steps afterwards when it returns to normal operation.
   */
  virtual bool exited_collision_avoidance(void) const = 0;

  /**
   * @brief If \ref exited_collision_avoidance() returns \c TRUE, then this
   * should return the duration of the collision avoidance in timesteps.
   */
  virtual uint collision_avoidance_duration(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_HPP_ */
