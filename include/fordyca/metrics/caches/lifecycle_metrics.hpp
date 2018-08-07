/**
 * @file lifecycle_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class lifecycle_metrics
 * @ingroup metrics caches
 *
 * @brief Interface defining lifecycle metrics that can be collected on
 * static/dynamic caches in the arena during their lifetime.
 */
class lifecycle_metrics : public rcppsw::metrics::base_metrics {
 public:
  lifecycle_metrics(void) = default;
  ~lifecycle_metrics(void) override = default;
  lifecycle_metrics(const lifecycle_metrics&) = default;
  lifecycle_metrics& operator=(const lifecycle_metrics&) = default;

  /**
   * @brief Should return the # caches that have been created in the arena this
   * timestep.
   */
  virtual uint caches_created(void) const = 0;

  /**
   * @brief Should return the # of caches that have been depleted in the arena
   * this timestep.
   */
  virtual uint caches_depleted(void) const = 0;
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_LIFECYCLE_METRICS_HPP_ */
