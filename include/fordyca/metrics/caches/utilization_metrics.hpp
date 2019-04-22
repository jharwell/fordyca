/**
 * @file utilization_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class utilization_metrics
 * @ingroup metrics caches
 *
 * @brief Interface defining utilization metrics that can be collected on
 * static/dynamic caches in the arena during their lifetime.
 */
class utilization_metrics : virtual public rmetrics::base_metrics {
 public:
  utilization_metrics(void) = default;
  ~utilization_metrics(void) override = default;
  utilization_metrics(const utilization_metrics&) = default;
  utilization_metrics& operator=(const utilization_metrics&) = default;

  /**
   * @brief Get the # of blocks currently in the cache (independent of any
   * calls to \ref reset_metrics()).
   */
  virtual size_t n_blocks(void) const = 0;

  /**
   * @brief Should return the # of blocks a given cache has had picked up from
   * it this timestep.
   *
   * This is currently always 1, due to limitations/shortcuts taken with the
   * block pickup events.
   */
  virtual uint total_block_pickups(void) const = 0;

  /**
   * @brief Should return the # of blocks a given cache has had dropped in it
   * this timestep.
   *
   * This is currently always 1, due to limitations/shortcuts taken with the
   * block drop events.
   */
  virtual uint total_block_drops(void) const = 0;
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_HPP_ */
