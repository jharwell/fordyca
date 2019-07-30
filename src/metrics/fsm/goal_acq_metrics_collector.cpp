/**
 * @file goal_acq_metrics_collector.cpp
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
#include "fordyca/metrics/fsm/goal_acq_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acq_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
goal_acq_metrics_collector::goal_acq_metrics_collector(const std::string& ofname,
                                                       uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> goal_acq_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_acquiring_goal",
    "cum_avg_acquiring_goal",
    "int_avg_vectoring_to_goal",
    "cum_avg_vectoring_to_goal",
    "int_avg_true_exploring_for_goal",
    "cum_avg_true_exploring_for_goal",
    "int_avg_false_exploring_for_goal",
    "cum_avg_false_exploring_for_goal",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void goal_acq_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void goal_acq_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::goal_acq_metrics&>(metrics);
  auto [is_exp, true_exp] = m.is_exploring_for_goal();

  m_interval.n_true_exploring_for_goal += static_cast<uint>(is_exp && true_exp);
  m_interval.n_false_exploring_for_goal +=
      static_cast<uint>(is_exp && !true_exp);
  m_interval.n_acquiring_goal +=
      static_cast<uint>(is_exp || m.is_vectoring_to_goal());
  m_interval.n_vectoring_to_goal += static_cast<uint>(m.is_vectoring_to_goal());

  m_cum.n_true_exploring_for_goal += static_cast<uint>(is_exp && true_exp);
  m_cum.n_false_exploring_for_goal += static_cast<uint>(is_exp && !true_exp);
  m_cum.n_acquiring_goal +=
      static_cast<uint>(is_exp || m.is_vectoring_to_goal());
  m_cum.n_vectoring_to_goal += static_cast<uint>(m.is_vectoring_to_goal());
} /* collect() */

bool goal_acq_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_intavg(m_interval.n_acquiring_goal.load());
  line += csv_entry_tsavg(m_cum.n_acquiring_goal.load());
  line += csv_entry_intavg(m_interval.n_vectoring_to_goal.load());
  line += csv_entry_tsavg(m_cum.n_vectoring_to_goal.load());
  line += csv_entry_intavg(m_interval.n_true_exploring_for_goal.load());
  line += csv_entry_tsavg(m_cum.n_true_exploring_for_goal.load());
  line += csv_entry_intavg(m_interval.n_false_exploring_for_goal.load());
  line += csv_entry_tsavg(m_cum.n_false_exploring_for_goal.load(), true);

  return true;
} /* store_foraging_stats() */

void goal_acq_metrics_collector::reset_after_interval(void) {
  m_interval.n_true_exploring_for_goal = 0;
  m_interval.n_false_exploring_for_goal = 0;
  m_interval.n_acquiring_goal = 0;
  m_interval.n_vectoring_to_goal = 0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
