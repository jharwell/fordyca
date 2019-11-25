/**
 * \file temporal_variance_metrics_collector.cpp
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
#include "fordyca/metrics/temporal_variance_metrics_collector.hpp"

#include "fordyca/metrics/temporal_variance_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
temporal_variance_metrics_collector::temporal_variance_metrics_collector(
    const std::string& ofname)
    : base_metrics_collector(ofname, 1) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> temporal_variance_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
      "swarm_motion_throttle",
      "env_block_manip",
      "env_cache_usage"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> temporal_variance_metrics_collector::csv_line_build(void) {
  std::string line;
  line += std::to_string(m_avg_motion_throttle) + separator();
  line += std::to_string(m_env_block_manip.v()) + separator();
  line += std::to_string(m_env_cache_usage.v());
  return boost::make_optional(line);
} /* csv_line_build() */

void temporal_variance_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const temporal_variance_metrics&>(metrics);
  m_avg_motion_throttle = m.avg_motion_throttle();
  m_env_block_manip = m.env_block_manipulation();
  m_env_cache_usage = m.env_cache_usage();
} /* collect() */

NS_END(metrics, fordyca);
