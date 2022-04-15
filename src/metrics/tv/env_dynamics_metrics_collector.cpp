/**
 * \file env_dynamics_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/tv/env_dynamics_metrics_collector.hpp"

#include "fordyca/metrics/tv/env_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tv);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
env_dynamics_metrics_collector::env_dynamics_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const env_dynamics_metrics&>(metrics);
  m_data.interval.avg_motion_throttle = m.avg_motion_throttle();
  m_data.interval.block_manip_penalty = m.arena_block_manip_penalty();
  m_data.interval.cache_usage_penalty = m.cache_usage_penalty();
} /* collect() */

void env_dynamics_metrics_collector::reset_after_interval(void) {
  m_data.interval.avg_motion_throttle = 0;
  m_data.interval.block_manip_penalty = rtypes::timestep{ 0 };
  m_data.interval.cache_usage_penalty = rtypes::timestep{ 0 };
} /* reset_after_interval() */

NS_END(tv, metrics, fordyca);
