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
#include "fordyca/metrics/collectors/robot_metrics/distance_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/distance_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors, robot_metrics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string distance_metrics_collector::csv_header_build(const std::string& header) {
  std::string line = "";
  for (size_t i = 0; i < m_n_robots; ++i) {
    line += "robot" + std::to_string(i) + separator();
  } /* for(i..) */

  return base_metric_collector::csv_header_build(header) + line;
} /* csv_header_build() */

void distance_metrics_collector::reset(void) {
  base_metric_collector::reset();
  m_stats.clear();
  for (size_t i = 0; i < m_n_robots; ++i) {
    m_stats.emplace_back();
  } /* for(i..) */
} /* reset() */

bool distance_metrics_collector::csv_line_build(std::string& line) {
  for (auto s : m_stats) {
    line += std::to_string(s.cum_distance) + separator();
  } /* for(s..) */
  return true;
} /* csv_line_build() */

void distance_metrics_collector::collect(
    const collectible_metrics::base_collectible_metrics& metrics) {
  auto& m = static_cast<const collectible_metrics::robot_metrics::distance_metrics&>(metrics);
  m_stats[m.entity_id()].cum_distance += m.timestep_distance();
} /* collect() */


NS_END(robot_metrics, collectors, metrics, fordyca);
