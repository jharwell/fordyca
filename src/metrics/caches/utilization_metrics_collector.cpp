/**
 * @file utilization_metrics_collector.cpp
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
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/caches/utilization_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_collector::utilization_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> utilization_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_blocks",
    "cum_avg_blocks",
    "int_avg_pickups",
    "cum_avg_pickups",
    "int_avg_drops" ,
    "cum_avg_drops" ,
    "int_avg_caches",
    "cum_avg_caches"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void utilization_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool utilization_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_domavg(m_stats.int_blocks, m_stats.int_cache_count);
  line += csv_entry_domavg(m_stats.cum_blocks, m_stats.cum_cache_count);
  line += csv_entry_domavg(m_stats.int_pickups, m_stats.int_cache_count);
  line += csv_entry_domavg(m_stats.cum_pickups, m_stats.cum_cache_count);
  line += csv_entry_domavg(m_stats.int_drops, m_stats.int_cache_count);
  line += csv_entry_domavg(m_stats.cum_drops, m_stats.cum_cache_count);
  line += csv_entry_intavg(m_stats.int_cache_count);
  line += csv_entry_tsavg(m_stats.cum_cache_count);
  return true;
} /* csv_line_build() */

void utilization_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const utilization_metrics&>(metrics);
  m_stats.int_pickups += m.total_block_pickups();
  m_stats.int_drops += m.total_block_drops();
  m_stats.int_blocks += m.n_blocks();

  m_stats.cum_pickups += m.total_block_pickups();
  m_stats.cum_drops += m.total_block_drops();
  m_stats.cum_blocks += m.n_blocks();

  ++m_stats.int_cache_count;
  ++m_stats.cum_cache_count;
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_stats.int_blocks = 0;
  m_stats.int_pickups = 0;
  m_stats.int_drops = 0;
  m_stats.int_blocks = 0;
  m_stats.int_cache_count = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
