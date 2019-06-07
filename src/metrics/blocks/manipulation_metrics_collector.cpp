/**
 * @file manipulation_metrics_collector.cpp
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
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_collector::manipulation_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> manipulation_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_free_pickup_events",
    "int_avg_free_drop_events",
    "int_avg_free_pickup_penalty",
    "int_avg_free_drop_penalty",
    "int_avg_cache_pickup_events",
    "int_avg_cache_drop_events",
    "int_avg_cache_pickup_penalty",
    "int_avg_cache_drop_penalty"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void manipulation_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool manipulation_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.int_free_pickup_events) + separator();
  line += std::to_string(m_stats.int_free_drop_events) + separator();

  line += csv_entry_domavg(m_stats.int_free_pickup_penalty,
                           m_stats.int_free_pickup_events);
  line += csv_entry_domavg(m_stats.int_free_drop_penalty,
                           m_stats.int_free_drop_events);

  line += std::to_string(m_stats.int_cache_pickup_events) + separator();
  line += std::to_string(m_stats.int_cache_drop_events) + separator();

  line += csv_entry_domavg(m_stats.int_cache_pickup_penalty,
                           m_stats.int_cache_pickup_events);
  line += csv_entry_domavg(m_stats.int_cache_drop_penalty,
                           m_stats.int_cache_drop_events);
  return true;
} /* csv_line_build() */

void manipulation_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const manipulation_metrics&>(metrics);
  if (m.free_pickup_event()) {
    ++m_stats.int_free_pickup_events;
    m_stats.int_free_pickup_penalty += m.penalty_served();
  } else if (m.free_drop_event()) {
    ++m_stats.int_free_drop_events;
    m_stats.int_free_drop_penalty += m.penalty_served();
  } else if (m.cache_pickup_event()) {
    ++m_stats.int_cache_pickup_events;
    m_stats.int_cache_pickup_penalty += m.penalty_served();
  } else if (m.cache_drop_event()) {
    ++m_stats.int_cache_drop_events;
    m_stats.int_cache_drop_penalty += m.penalty_served();
  }
} /* collect() */

void manipulation_metrics_collector::reset_after_interval(void) {
  m_stats = {0, 0, 0, 0, 0, 0, 0, 0};
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
