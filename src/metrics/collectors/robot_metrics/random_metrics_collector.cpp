/**
 * @file random_metrics_collector.cpp
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
#include "fordyca/metrics/collectors/robot_metrics/random_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/random_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors, robot_metrics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string random_metrics_collector::csv_header_build(const std::string& header) {
  return base_metric_collector::csv_header_build(header) +
      "n_exploring_for_block;n_avoiding_collision;n_transporting_to_nest";
} /* csv_header_build() */

void random_metrics_collector::reset(void) {
  base_metric_collector::reset();
  reset_on_timestep();
} /* reset() */

void random_metrics_collector::collect(
    const collectible_metrics::robot_metrics::random_metrics& metrics) {
  m_stats.n_exploring_for_block += metrics.is_exploring_for_block();
  m_stats.n_transporting_to_nest += metrics.is_transporting_to_nest();
  m_stats.n_avoiding_collision += metrics.is_avoiding_collision();
} /* collect() */

bool random_metrics_collector::csv_line_build(std::string& line) {
  line = std::to_string(m_stats.n_exploring_for_block) + ";" +
         std::to_string(m_stats.n_avoiding_collision) + ";" +
         std::to_string(m_stats.n_transporting_to_nest);
  return true;
} /* store_foraging_stats() */

void random_metrics_collector::reset_on_timestep(void) {
  m_stats = {0, 0, 0};
} /* reset_on_timestep() */

NS_END(robot_metrics, collectors, metrics, fordyca);
