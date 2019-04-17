/**
 * @file goal_acquisition_metrics_collector.cpp
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
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
goal_acquisition_metrics_collector::goal_acquisition_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval), m_stats() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> goal_acquisition_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_acquiring_goal",
    "cum_avg_acquiring_goal",
    "int_avg_vectoring_to_goal",
    "cum_avg_vectoring_to_goal",
    "int_avg_exploring_for_goal",
    "cum_avg_exploring_for_goal",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void goal_acquisition_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void goal_acquisition_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(metrics);
  m_stats.n_int_exploring_for_goal +=
      static_cast<uint>(m.is_exploring_for_goal());
  m_stats.n_int_acquiring_goal +=
      static_cast<uint>(m.is_exploring_for_goal() || m.is_vectoring_to_goal());
  m_stats.n_int_vectoring_to_goal += static_cast<uint>(m.is_vectoring_to_goal());

  m_stats.n_cum_exploring_for_goal +=
      static_cast<uint>(m.is_exploring_for_goal());
  m_stats.n_cum_acquiring_goal +=
      static_cast<uint>(m.is_exploring_for_goal() || m.is_vectoring_to_goal());
  m_stats.n_cum_vectoring_to_goal += static_cast<uint>(m.is_vectoring_to_goal());
} /* collect() */

bool goal_acquisition_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_intavg(m_stats.n_int_acquiring_goal);
  line += csv_entry_tsavg(m_stats.n_cum_acquiring_goal);
  line += csv_entry_intavg(m_stats.n_int_vectoring_to_goal);
  line += csv_entry_tsavg(m_stats.n_cum_vectoring_to_goal);
  line += csv_entry_intavg(m_stats.n_int_exploring_for_goal);
  line += csv_entry_tsavg(m_stats.n_cum_exploring_for_goal);
  return true;
} /* store_foraging_stats() */

void goal_acquisition_metrics_collector::reset_after_interval(void) {
  m_stats.n_int_exploring_for_goal = 0;
  m_stats.n_int_acquiring_goal = 0;
  m_stats.n_int_vectoring_to_goal = 0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
