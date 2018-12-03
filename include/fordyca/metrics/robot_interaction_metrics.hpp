/**
 * @file robot_interaction_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class robot_interaction_metrics
 * @ingroup metrics
 *
 * @brief Defines the metrics to be collected regarding robot interaction in the
 * arena. Based on Szabo2014.
 *
 * Metrics are collected every timestep.
 */
class robot_interaction_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  robot_interaction_metrics(void) = default;

  /**
   * @brief Vector of distances (one per robot), to each robot's closest
   * neighbor.
   */
  virtual std::vector<double> nearest_neighbors(void) const = 0;
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_HPP_ */
