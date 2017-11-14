/**
 * @file block_metrics_collector.cpp
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
#include "fordyca/metrics/collectors/block_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/block_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block_metrics_collector::csv_header_build(const std::string& header) {
  return base_metric_collector::csv_header_build(header) +
      "collected_blocks;avg_carries";
} /* csv_header_build() */

void block_metrics_collector::reset(void) {
  base_metric_collector::reset();
  m_metrics = {0, 0, 0};
} /* reset() */

bool block_metrics_collector::csv_line_build(std::string& line) {
  double avg_carries = 0;
  if (m_metrics.block_carries > 0) {
    avg_carries = static_cast<double>(m_metrics.total_collected/
                                      m_metrics.total_carries);
    line = std::to_string(m_metrics.total_collected) + ";" +
           std::to_string(avg_carries) + ";";
    m_metrics.block_carries = 0;
    return true;
  }
  return false;
} /* csv_line_build() */

void block_metrics_collector::collect(
    const collectible_metrics::block_metrics& metrics) {
  ++m_metrics.total_collected;
  m_metrics.block_carries = metrics.n_carries();
  m_metrics.total_carries += metrics.n_carries();
} /* collect() */

NS_END(collectors, metrics, fordyca);
