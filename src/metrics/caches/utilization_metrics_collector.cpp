/**
 * @file utilization_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/caches/utilization_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_collector::utilization_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string utilization_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  return base_metrics_collector::csv_header_build(header) +
      "avg_blocks" + separator() +
      "avg_pickups" + separator() +
      "avg_drops"  + separator() +
      "avg_caches" + separator();
  // clang-format on
} /* csv_header_build() */

void utilization_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool utilization_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  /*
   * Because the # of caches can change at any time in the arena, we need to use
   * the count of all caches that are involved in the events that we collect
   * metrics on and use that to average the cumulative counts that we get.
   */
  line += (!m_cache_ids.empty())
              ? std::to_string(static_cast<double>(m_stats.n_blocks) /
                               (m_cache_ids.size() * interval()))
              : "0";
  line += separator();

  line += (!m_cache_ids.empty())
              ? std::to_string(static_cast<double>(m_stats.n_pickups) /
                               (m_cache_ids.size()))
              : "0";
  line += separator();

  line += (!m_cache_ids.empty())
              ? std::to_string(static_cast<double>(m_stats.n_drops) /
                               (m_cache_ids.size()))
              : "0";
  line += separator();

  line += std::to_string(m_cache_ids.size()) + separator();
  return true;
} /* csv_line_build() */

void utilization_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const utilization_metrics&>(metrics);
  m_stats.n_blocks += m.n_blocks();
  m_cache_ids.insert(m.cache_id());
  m_stats.n_pickups += m.total_block_pickups();
  m_stats.n_drops += m.total_block_drops();
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_stats.n_blocks = 0;
  m_stats.n_pickups = 0;
  m_stats.n_drops = 0;
  m_cache_ids.clear();
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
