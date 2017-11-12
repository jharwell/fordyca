/**
 * @file block_stat_collector.cpp
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
#include "fordyca/diagnostics/block_stat_collector.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block_stat_collector::csv_header_build(const std::string& header) {
  return base_stat_collector::csv_header_build(header) +
      "collected_blocks;avg_carries";
} /* csv_header_build() */

void block_stat_collector::reset(void) {
  base_stat_collector::reset();
  m_stats = {0, 0, 0};
} /* reset() */

bool block_stat_collector::csv_line_build(std::string& line) {
  double avg_carries = 0;
  if (m_stats.block_carries > 0) {
    avg_carries = static_cast<double>(m_stats.total_collected/
                                      m_stats.total_carries);
    line = std::to_string(m_stats.total_collected) + ";" +
           std::to_string(avg_carries) + ";";
    return true;
  }
  return false;
} /* csv_line_build() */

void block_stat_collector::collect(const carryable_object_diagnostics& block) {
  ++m_stats.total_collected;
  m_stats.block_carries = block.n_carries();
  m_stats.total_carries += block.n_carries();
} /* collect() */


NS_END(diagnostics, fordyca);
