/**
 * @file movement_metrics_collector.cpp
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
#include "fordyca/metrics/fsm/movement_metrics_collector.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
movement_metrics_collector::movement_metrics_collector(const std::string& ofname,
                                                       uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string movement_metrics_collector::csv_header_build(
    const std::string& header) {
  return base_metrics_collector::csv_header_build(header) +
      "int_avg_distance" + separator() +
      "cum_avg_distance" + separator() +
      "int_avg_velocity" + separator() +
      "cum_avg_velocity" + separator();
} /* csv_header_build() */

void movement_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool movement_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.int_distance /
                         static_cast<double>(m_stats.int_robot_count)) + separator();
  line += std::to_string(m_stats.cum_distance /
                         static_cast<double>(m_stats.cum_robot_count)) + separator();
  line += std::to_string(m_stats.int_velocity /
                         static_cast<double>(m_stats.int_robot_count)) + separator();
  line += std::to_string(m_stats.cum_velocity /
                         static_cast<double>(m_stats.cum_robot_count)) + separator();
          separator();
  return true;
} /* csv_line_build() */

void movement_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::movement_metrics&>(metrics);
  ++m_stats.int_robot_count;
  ++m_stats.cum_robot_count;
  m_stats.cum_distance += m.distance();
  m_stats.int_distance += m.distance();
  m_stats.cum_velocity += m.velocity().Length();
  m_stats.int_velocity += m.velocity().Length();
} /* collect() */

void movement_metrics_collector::reset_after_interval(void) {
  m_stats.int_distance = 0.0;
  m_stats.int_velocity = 0.0;
  m_stats.int_robot_count = 0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
