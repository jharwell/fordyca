/**
 * @file swarm_dist2D_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_DIST2D_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_DIST2D_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/metrics/base_metrics.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class swarm_dist2D_metrics
 * @ingroup fordyca metrics spatial
 *
 * @brief Defines the metrics to be collected from swarms regarding spatial
 * distributions of things within 2D space.
 *
 * Metrics are collected every timestep.
 */
class swarm_dist2D_metrics : public virtual rmetrics::base_metrics {
 public:
  swarm_dist2D_metrics(void) = default;

  /**
   * @brief Return the robot's current position in 2D space in real
   * coordinates.
   */
  virtual const rmath::vector2d& position2D(void) const = 0;

  /**
   * @brief Return the robot's discretized position in 2D space.
   */
  virtual const rmath::vector2u& discrete_position2D(void) const = 0;

  /**
   * @brief Return the robot's current heading in 2D space.
   */
  virtual rmath::vector2d heading2D(void) const = 0;
};

NS_END(spatial, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_DIST2D_METRICS_HPP_ */
