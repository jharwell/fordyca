/**
 * @file arena_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_ARENA_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_ARENA_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class arena_metrics
 * @ingroup metrics
 *
 * @brief Defines the metrics to be collected from the arena mapn.
 *
 * Metrics are collected every timestep.
 */
class arena_metrics : virtual public rcppsw::metrics::base_metrics {
 public:
  arena_metrics(void) = default;

  /**
   * @brief Should return \c TRUE iff there is currently a robot is the cell at
   * (i,j) in the arena.
   */
  virtual bool has_robot(size_t i, size_t j) const = 0;
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ARENA_METRICS_HPP_ */
