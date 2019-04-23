/**
 * @file grid2D_avg_metrics_collector.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/metrics/grid2D_avg_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
grid2D_avg_metrics_collector::grid2D_avg_metrics_collector(
    const std::string& ofname,
    uint interval,
    const rmath::vector2u& dims)
    : base_metrics_collector(ofname, interval, true),
      m_stats(dims.x(), dims.y()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> grid2D_avg_metrics_collector::csv_header_cols(void) const {
  std::list<std::string> cols;
  for (size_t j = 0; j < m_stats.ysize(); ++j) {
    cols.push_back("y" + std::to_string(j));
  } /* for(j..) */

  return cols;
} /* csv_header_cols() */

void grid2D_avg_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool grid2D_avg_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  for (size_t i = 0; i < m_stats.xsize(); ++i) {
    for (size_t j = 0; j < m_stats.ysize(); ++j) {
      line +=
          std::to_string(m_stats.access(i, j) / static_cast<double>(m_count)) +
          separator();
    } /* for(j..) */
    line += "\n";
  } /* for(i..) */

  return true;
} /* csv_line_build() */

void grid2D_avg_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  ++m_count;
  for (size_t i = 0; i < m_stats.xsize(); ++i) {
    for (size_t j = 0; j < m_stats.ysize(); ++j) {
      m_stats.access(i, j) += collect_cell(metrics, rmath::vector2u(i, j));
    } /* for(j..) */
  }   /* for(i..) */
} /* collect() */

NS_END(metrics, fordyca);
