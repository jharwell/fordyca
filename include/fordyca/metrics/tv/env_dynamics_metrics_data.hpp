/**
 * \file env_dynamics_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_DATA_HPP_
#define INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_DATA_HPP_

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

#endif /* INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_DATA_HPP_ */
