/**
 * \file env_dynamics_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, tv, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct env_dynamics_metrics_data
 * \ingroup metrics tv detail
 *
 * \brief Container for holding collected statistics of \ref
 * env_dynamics_metrics.
 */
struct env_dynamics_metrics_data {
  double           avg_motion_throttle{0.0};
  rtypes::timestep block_manip_penalty{0};
  rtypes::timestep cache_usage_penalty{0};
};

NS_END(detail);

struct env_dynamics_metrics_data : public rmetrics::base_data {
  detail::env_dynamics_metrics_data interval{};
};

NS_END(tv, metrics, fordyca);

