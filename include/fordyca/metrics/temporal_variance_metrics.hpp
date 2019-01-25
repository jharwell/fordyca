/**
 * @file temporal_variance_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_HPP_

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
 * @class temporal_variance_metrics
 * @ingroup metrics
 *
 * @brief Defines the metrics to be collected from the environment and the swarm
 * about the different types of temporal variance that can be applied to
 * each.
 *
 * Not really "metrics" per-se, but more of a way to record variances for later
 * usage in post-processing.
 *
 * Metrics are collected and output every timestep.
 */
class temporal_variance_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  temporal_variance_metrics(void) = default;

  /**
   * @brief Return the average motion throttling within the swarm, as a
   * percentage [0, 1.0].
   */
  virtual double swarm_motion_throttle(void) const = 0;

  /**
   * @brief Return the current value of the block manipulation penalty present
   * in the environment.
   */
  virtual double env_block_manipulation(void) const = 0;

  /**
   * @brief Return the current value of the cache usage penalty present in the
   * environment.
   */
  virtual double env_cache_usage(void) const = 0;
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_HPP_ */
