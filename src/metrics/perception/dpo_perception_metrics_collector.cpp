/**
 * @file dpo_perception_metrics_collector.cpp
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
#include "fordyca/metrics/perception/dpo_perception_metrics_collector.hpp"
#include <numeric>

#include "fordyca/metrics/perception/dpo_perception_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_perception_metrics_collector::dpo_perception_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> dpo_perception_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
      "int_avg_known_blocks",
      "cum_avg_known_blocks",
      "int_avg_known_caches",
      "cum_avg_known_caches",
      "int_avg_block_pheromone_density",
      "cum_avg_block_pheromone_density",
      "int_avg_cache_pheromone_density",
      "cum_avg_cache_pheromone_density"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void dpo_perception_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool dpo_perception_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_domavg(m_int_known_blocks, m_int_robot_count);
  line += csv_entry_domavg(m_cum_known_blocks, m_cum_robot_count);
  line += csv_entry_domavg(m_int_known_caches, m_int_robot_count);
  line += csv_entry_domavg(m_cum_known_caches, m_cum_robot_count);

  line += csv_entry_domavg(m_int_block_density_sum.v(), m_int_robot_count);
  line += csv_entry_domavg(m_cum_block_density_sum.v(), m_cum_robot_count);
  line += csv_entry_domavg(m_int_cache_density_sum.v(), m_int_robot_count);
  line += csv_entry_domavg(m_cum_cache_density_sum.v(), m_cum_robot_count, true);
  return true;
} /* csv_line_build() */

void dpo_perception_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const dpo_perception_metrics&>(metrics);
  ++m_int_robot_count;
  ++m_cum_robot_count;
  m_int_known_blocks += m.n_known_blocks();
  m_int_known_caches += m.n_known_caches();
  m_int_block_density_sum += m.avg_block_density();
  m_int_cache_density_sum += m.avg_cache_density();

  m_cum_known_blocks += m.n_known_blocks();
  m_cum_known_caches += m.n_known_caches();
  m_cum_block_density_sum += m.avg_block_density();
  m_cum_cache_density_sum += m.avg_cache_density();
} /* collect() */

void dpo_perception_metrics_collector::reset_after_interval(void) {
  m_int_robot_count = 0;
  m_int_known_blocks = 0;
  m_int_known_caches = 0;
  m_int_block_density_sum.reset();
  m_int_cache_density_sum.reset();
} /* reset_after_interval() */

NS_END(perception, metrics, fordyca);
