/**
 * @file cache_metrics_collector.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/metrics/collectors/cache_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/cache_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string cache_metrics_collector::csv_header_build(const std::string& header) {
  return base_metric_collector::csv_header_build(header) +
      "total_blocks" + separator() +
      "total_pickups"  + separator() +
      "total_drops"  + separator();
} /* csv_header_build() */

void cache_metrics_collector::reset(void) {
  base_metric_collector::reset();
  m_stats = {0, 0, 0};
} /* reset() */

bool cache_metrics_collector::csv_line_build(std::string& line) {
  if (!m_new_data) {
    return false;
  }
  line = std::to_string(m_stats.total_blocks) + separator() +
         std::to_string(m_stats.total_pickups) + separator() +
         std::to_string(m_stats.total_drops) + separator();
  m_new_data = false;
  return true;
} /* csv_line_build() */

void cache_metrics_collector::collect(
    const collectible_metrics::cache_metrics& metrics) {

  m_stats.total_blocks += metrics.n_blocks();
  m_stats.total_pickups += metrics.n_block_pickups();
  m_stats.total_drops += metrics.n_block_drops();
  m_new_data = true;
} /* collect() */


NS_END(collectors, metrics, fordyca);
