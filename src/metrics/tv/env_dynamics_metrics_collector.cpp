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
    const std::string& ofname_stem)
    : base_metrics_collector(ofname_stem,
                             rtypes::timestep(1),
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> env_dynamics_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
      "swarm_motion_throttle",
      "block_manip_penalty",
      "cache_usage_penalty"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> env_dynamics_metrics_collector::csv_line_build(void) {
  std::string line;
  line += rcppsw::to_string(m_avg_motion_throttle) + separator();
  line += rcppsw::to_string(m_block_manip_penalty) + separator();
  line += rcppsw::to_string(m_cache_usage_penalty);
  return boost::make_optional(line);
} /* csv_line_build() */

void env_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const env_dynamics_metrics&>(metrics);
  m_avg_motion_throttle = m.avg_motion_throttle();
  m_block_manip_penalty = m.block_manip_penalty();
  m_cache_usage_penalty = m.cache_usage_penalty();
} /* collect() */

NS_END(tv, metrics, fordyca);
