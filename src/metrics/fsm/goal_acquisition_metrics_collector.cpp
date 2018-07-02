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
std::string goal_acquisition_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  return base_metrics_collector::csv_header_build(header) +
      "avg_acquiring_goal" + separator() +
      "avg_cum_acquiring_goal" + separator() +
      "avg_vectoring_to_goal" + separator() +
      "avg_cum_vectoring_to_goal" + separator() +
      "avg_exploring_for_goal" + separator() +
      "avg_cum_exploring_for_goal" + separator();
  // clang-format on
} /* csv_header_build() */

void goal_acquisition_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void goal_acquisition_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(metrics);
  m_stats.n_exploring_for_goal += static_cast<uint>(m.is_exploring_for_goal());
  m_stats.n_acquiring_goal +=
      static_cast<uint>(m.is_exploring_for_goal() || m.is_vectoring_to_goal());
  m_stats.n_vectoring_to_goal += static_cast<uint>(m.is_vectoring_to_goal());

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
  line = std::to_string(m_stats.n_acquiring_goal /
                        static_cast<double>(interval())) +
         separator() +
         std::to_string(m_stats.n_cum_acquiring_goal /
                        static_cast<double>(timestep() + 1)) +
         separator() +
         std::to_string(m_stats.n_vectoring_to_goal /
                        static_cast<double>(interval())) +
         separator() +
         std::to_string(m_stats.n_cum_vectoring_to_goal /
                        static_cast<double>(timestep() + 1)) +
         separator() +
         std::to_string(m_stats.n_exploring_for_goal /
                        static_cast<double>(interval())) +
         separator() +
         std::to_string(m_stats.n_cum_exploring_for_goal /
                        static_cast<double>(timestep() + 1)) +
         separator();
  return true;
} /* store_foraging_stats() */

void goal_acquisition_metrics_collector::reset_after_interval(void) {
  m_stats.n_exploring_for_goal = 0;
  m_stats.n_acquiring_goal = 0;
  m_stats.n_vectoring_to_goal = 0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
