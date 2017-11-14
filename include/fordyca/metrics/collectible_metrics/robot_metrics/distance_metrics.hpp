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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_DISTANCE_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_DISTANCE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/collectible_metrics/base_collectible_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectible_metrics, robot_metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class distance_metrics : public base_collectible_metrics {
 public:
  distance_metrics(void) {}
  virtual ~distance_metrics(void) {}

  virtual double timestep_distance(void) const = 0;
  virtual size_t entity_id(void) const = 0;
};

NS_END(robot_metrics, collectible_metrics, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_DISTANCE_METRICS_HPP_ */
