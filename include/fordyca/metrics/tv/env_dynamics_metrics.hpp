/**
 * \file env_dynamics_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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

