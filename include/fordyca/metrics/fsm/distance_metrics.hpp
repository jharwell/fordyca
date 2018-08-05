/**
 * @file distance_metrics.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_DISTANCE_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_DISTANCE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class distance_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics regarding distance traveled should be
 * collected from all robots.
 */
class distance_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  distance_metrics(void) = default;
  ~distance_metrics(void) override = default;

  /**
   * @brief Get the distance that a robot has traveled in a single timestep.
   *
   * This will be called every timestep by the \ref distance_metrics_collector
   * on all robots.
   */
  virtual double timestep_distance(void) const = 0;

  /**
   * @brief Get the ID of a robot, for use in associating gathered metrics with
   * a specific robot.
   */
  virtual int entity_id(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_DISTANCE_METRICS_HPP_ */
