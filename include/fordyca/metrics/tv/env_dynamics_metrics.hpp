/**
 * \file env_dynamics_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/metrics/base_env_dynamics_metrics.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_metrics
 * \ingroup metrics tv
 *
 * \brief Defines the metrics to be collected from the environment and the swarm
 * about the different types of environmental variance that can be applied to
 * each.
 *
 * Not really "metrics" per-se, but more of a way to record variances for later
 * usage in post-processing.
 *
 * Metrics are collected and output EVERY timestep.
 */
class env_dynamics_metrics : public ctv::metrics::base_env_dynamics_metrics {
 public:
  env_dynamics_metrics(void) = default;

  /**
   * \brief Return the current value of the cache usage penalty present in the
   * environment.
   */
  virtual rtypes::timestep cache_usage_penalty(void) const = 0;
};

NS_END(tv, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_HPP_ */
