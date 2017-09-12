/**
 * @file stat_collector.cpp
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
#include "fordyca/support/base_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_stat_collector::csv_line_write(uint timestep) {
  m_ofile << std::to_string(timestep) + ";" +
      csv_line_build() << std::endl;
} /* csv_line_write() */

void base_stat_collector::csv_header_write(void) {
  std::string header = csv_header_build("");
  m_ofile << header + "\n";
} /* csv_header_write() */

std::string base_stat_collector::csv_header_build(const std::string& header) {
  return header + "clock;";
} /* csv_header_build() */

void base_stat_collector::reset(void) {
  /* Open output file and truncate */
  if (m_ofile.is_open()) {
    m_ofile.close();
  }
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);
  csv_header_write();
} /* reset() */

NS_END(support, fordyca);
