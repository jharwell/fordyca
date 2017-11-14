/**
 * @file random_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_RANDOM_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_RANDOM_METRICS_HPP_

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
class random_metrics : public base_collectible_metrics {
 public:
  random_metrics(void) {}
  virtual ~random_metrics(void) {}

  virtual bool is_exploring_for_block(void) const = 0;
  virtual bool is_avoiding_collision(void) const = 0;
  virtual bool is_transporting_to_nest(void) const = 0;
};

NS_END(robot_metrics, collectible_metrics, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_ROBOT_METRICS_RANDOM_METRICS_HPP_ */
