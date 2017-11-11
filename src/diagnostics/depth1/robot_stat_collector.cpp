/**
 * @file robot_stat_collector.cpp
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
#include "fordyca/diagnostics/depth1/robot_stat_collector.hpp"
#include "fordyca/diagnostics/depth1/collectible_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string robot_stat_collector::csv_header_build(const std::string& header) {
  return base_stat_collector::csv_header_build(header) +
      "n_exploring_for_block;n_avoiding_collision;n_transporting_to_nest;n_acquiring_block;n_vectoring_to_block";
} /* csv_header_build() */

void robot_stat_collector::reset(void) {
  base_stat_collector::reset();
  reset_on_timestep();
} /* reset() */

void robot_stat_collector::collect(const collectible_diagnostics& diagnostics) {
  m_stats.n_exploring_for_cache += diagnostics.is_exploring_for_cache();
  m_stats.n_acquiring_cache += diagnostics.is_acquiring_cache();
  m_stats.n_vectoring_to_cache += diagnostics.is_vectoring_to_cache();
  m_stats.n_transporting_to_cache += diagnostics.is_transporting_to_cache();
} /* collect() */

std::string robot_stat_collector::csv_line_build(void) {
  return std::to_string(m_stats.n_exploring_for_cache) + ";" +
      std::to_string(m_stats.n_acquiring_cache) + ";" +
      std::to_string(m_stats.n_vectoring_to_cache) + ";" +
      std::to_string(m_stats.n_vectoring_to_cache) + ";" +
      std::to_string(m_stats.n_transporting_to_cache);
} /* store_foraging_stats() */

void robot_stat_collector::reset_on_timestep(void) {
  m_stats = {0, 0, 0, 0};
} /* reset_on_timestep() */

NS_END(depth1, diagnostics, fordyca);
