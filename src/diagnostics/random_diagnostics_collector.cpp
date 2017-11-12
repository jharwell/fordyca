/**
 * @file random_diagnostics_collector.cpp
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
#include "fordyca/diagnostics/random_diagnostics_collector.hpp"
#include "fordyca/diagnostics/random_collectible_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string random_diagnostics_collector::csv_header_build(const std::string& header) {
  return base_stat_collector::csv_header_build(header) +
      "n_exploring_for_block;n_avoiding_collision;n_transporting_to_nest";
} /* csv_header_build() */

void random_diagnostics_collector::reset(void) {
  base_stat_collector::reset();
  reset_on_timestep();
} /* reset() */

void random_diagnostics_collector::collect(const random_collectible_diagnostics& diag) {
  m_stats.n_exploring_for_block += diag.is_exploring_for_block();
  m_stats.n_transporting_to_nest += diag.is_transporting_to_nest();
  m_stats.n_avoiding_collision += diag.is_avoiding_collision();
} /* collect() */

std::string random_diagnostics_collector::csv_line_build(void) {
  return std::to_string(m_stats.n_exploring_for_block) + ";" +
      std::to_string(m_stats.n_avoiding_collision) + ";" +
      std::to_string(m_stats.n_transporting_to_nest);
} /* store_foraging_stats() */

void random_diagnostics_collector::reset_on_timestep(void) {
  m_stats = {0, 0, 0};
} /* reset_on_timestep() */

NS_END(diagnostics, fordyca);
