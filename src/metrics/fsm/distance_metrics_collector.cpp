/**
 * @file distance_metrics_collector.cpp
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
#include "fordyca/metrics/fsm/distance_metrics_collector.hpp"
#include "fordyca/metrics/fsm/distance_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
distance_metrics_collector::distance_metrics_collector(const std::string& ofname,
                                                       uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string distance_metrics_collector::csv_header_build(
    const std::string& header) {
  return base_metrics_collector::csv_header_build(header) +
      "avg_distance" + separator() +
      "avg_cum_distance" + separator();
} /* csv_header_build() */

void distance_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool distance_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.distance) + separator();
  line += std::to_string(m_stats.cum_distance /
                         (static_cast<double>(timestep() + 1) / interval())) +
          separator();
  return true;
} /* csv_line_build() */

void distance_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::distance_metrics&>(metrics);
  m_stats.cum_distance += m.timestep_distance();
  m_stats.distance += m.timestep_distance();
} /* collect() */

void distance_metrics_collector::reset_after_interval(void) {
  m_stats.distance = 0.0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
