/**
 * @file collector.cpp
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
#include "fordyca/diagnostics/depth0/collector.hpp"
#include "fordyca/diagnostics/depth0/collectible_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics, depth0);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string collector::csv_header_build(const std::string& header) {
  return base_stat_collector::csv_header_build(header) +
      "n_acquiring_block;n_vectoring_to_block";
} /* csv_header_build() */

void collector::reset(void) {
  base_stat_collector::reset();
  reset_on_timestep();
} /* reset() */

void collector::collect(const collectible_diagnostics& diag) {
  m_stats.n_acquiring_block += diag.is_acquiring_block();
  m_stats.n_vectoring_to_block += diag.is_vectoring_to_block();
} /* collect() */

std::string collector::csv_line_build(void) {
  return std::to_string(m_stats.n_acquiring_block) + ";" +
      std::to_string(m_stats.n_vectoring_to_block);
} /* store_foraging_stats() */

void collector::reset_on_timestep(void) {
  m_stats = {0, 0};
} /* reset_on_timestep() */


NS_END(depth0, diagnostics, fordyca);
