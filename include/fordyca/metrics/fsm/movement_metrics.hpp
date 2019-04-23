/**
 * @file movement_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class movement_metrics
 * @ingroup fordyca metrics fsm
 *
 * @brief Interface defining what metrics regarding movement traveled should be
 * collected from all robots.
 */
class movement_metrics : virtual public rcppsw::metrics::base_metrics {
 public:
  movement_metrics(void) = default;
  ~movement_metrics(void) override = default;

  /**
   * @brief Get the movement that a robot has traveled in a single timestep.
   *
   * This will be called every timestep by the \ref movement_metrics_collector
   * on all robots.
   */
  virtual double distance(void) const = 0;

  /**
   * @brief Get the velocity that a robot has on a single timestep.
   */
  virtual rmath::vector2d velocity(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_HPP_ */
