/**
 * @file arena_metrics_collector.cpp
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
#include "fordyca/metrics/arena_metrics_collector.hpp"
#include <iostream>
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/metrics/arena_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_metrics_collector::arena_metrics_collector(const std::string& ofname,
                                                 uint interval,
                                                 const rcppsw::math::dcoord2& dims)
    : base_metrics_collector(ofname, interval, true),
      m_stats(dims.first, dims.second) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string arena_metrics_collector::csv_header_build(const std::string&) {
  std::string line;
  for (size_t j = 0; j < m_stats.ysize(); ++j) {
    line += "y" + std::to_string(j) + separator();
  } /* for(j..) */

  return line;
} /* csv_header_build() */

void arena_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool arena_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  for (size_t i = 0; i < m_stats.xsize(); ++i) {
    for (size_t j = 0; j < m_stats.ysize(); ++j) {
      line +=
          std::to_string(m_stats.access(i, j) / static_cast<double>(m_total)) +
          separator();
    } /* for(j..) */
    line += "\n";
  } /* for(i..) */

  return true;
} /* csv_line_build() */

void arena_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const arena_metrics&>(metrics);

  for (size_t i = 0; i < m_stats.xsize(); ++i) {
    for (size_t j = 0; j < m_stats.ysize(); ++j) {
      m_total += m.has_robot(i, j);
      m_stats.access(i, j) += m.has_robot(i, j);
    } /* for(j..) */
  }   /* for(i..) */
} /* collect() */

NS_END(metrics, fordyca);
