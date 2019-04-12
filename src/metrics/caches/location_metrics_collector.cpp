/**
 * @file location_metrics_collector.cpp
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
#include "fordyca/metrics/caches/location_metrics_collector.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/metrics/caches/location_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
location_metrics_collector::location_metrics_collector(
    const std::string& ofname,
    uint interval,
    const rmath::vector2u& dims)
    : base_metrics_collector(ofname, interval, true),
      m_stats(dims.x(), dims.y()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> location_metrics_collector::csv_header_cols(void) const {
  std::list<std::string> cols;
  for (size_t j = 0; j < m_stats.ysize(); ++j) {
    cols.push_back("y" + std::to_string(j));
  } /* for(j..) */

  return cols;
} /* csv_header_cols() */

void location_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool location_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  for (size_t i = 0; i < m_stats.xsize(); ++i) {
    for (size_t j = 0; j < m_stats.ysize(); ++j) {
      line += csv_entry_domavg(m_stats.access(i, j), m_total);
    } /* for(j..) */
    line += "\n";
  } /* for(i..) */

  return true;
} /* csv_line_build() */

void location_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const location_metrics&>(metrics);

  ++m_total;
  ++m_stats.access(m.location().x(), m.location().y());
} /* collect() */

NS_END(caches, metrics, fordyca);
