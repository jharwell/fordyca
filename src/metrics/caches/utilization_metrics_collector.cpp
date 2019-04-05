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
  /* clang-format off */
  return base_metrics_collector::csv_header_build(header) +
      "int_avg_blocks" + separator() +
      "cum_avg_blocks" + separator() +
      "int_avg_pickups" + separator() +
      "cum_avg_pickups" + separator() +
      "int_avg_drops"  + separator() +
      "cum_avg_drops"  + separator() +
      "int_avg_caches" + separator() +
      "cum_avg_caches" + separator();
  /* clang-format on */
} /* csv_header_build() */

void utilization_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool utilization_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(static_cast<double>(m_stats.int_blocks) / interval()) +
          separator();

  line += std::to_string(static_cast<double>(m_stats.cum_blocks) /
                         (timestep() + 1)) + separator();

  line += std::to_string(static_cast<double>(m_stats.int_pickups) / interval()) +
          separator();
  line += std::to_string(static_cast<double>(m_stats.cum_pickups) /
                         (timestep() + 1)) + separator();

  line += std::to_string(static_cast<double>(m_stats.int_drops) / interval()) +
          separator();
  line += std::to_string(static_cast<double>(m_stats.cum_drops) /
                         (timestep() + 1)) + separator();

  line += std::to_string(static_cast<double>(m_int_cache_count) /
                         interval()) + separator();
  line += std::to_string(static_cast<double>(m_cum_cache_count) /
                         (timestep() + 1)) + separator();
  return true;
} /* csv_line_build() */

void utilization_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const utilization_metrics&>(metrics);
  m_stats.int_pickups += m.total_block_pickups();
  m_stats.int_drops += m.total_block_drops();
  m_stats.int_blocks += m.n_blocks();

  m_stats.cum_pickups += m.total_block_pickups();
  m_stats.cum_drops += m.total_block_drops();
  m_stats.cum_blocks += m.n_blocks();

  ++m_int_cache_count;
  ++m_cum_cache_count;
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_stats.int_blocks = 0;
  m_stats.int_pickups = 0;
  m_stats.int_drops = 0;
  m_stats.int_blocks = 0;
  m_int_cache_count = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
