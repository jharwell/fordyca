/**
 * @file location_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class location_metrics
 * @ingroup fordyca metrics caches
 *
 * @brief Defines the metrics to be collected from a cache regarding its
 * location in the arena.
 *
 * Metrics are collected every timestep.
 */
class location_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  location_metrics(void) = default;

  /**
   * @brief Should return the discrete location of the cache.
   */
  virtual rcppsw::math::vector2u location(void) const = 0;
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_HPP_ */
