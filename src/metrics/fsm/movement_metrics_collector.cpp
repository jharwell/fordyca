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
std::list<std::string> movement_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_distance",
    "cum_avg_distance",
    "int_avg_velocity",
    "cum_avg_velocity"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void movement_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool movement_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_domavg(m_interval.distance.load(), m_interval.robot_count);
  line += csv_entry_domavg(m_cum.distance.load(), m_cum.robot_count);

  line += csv_entry_domavg(m_interval.velocity.load(), m_interval.robot_count);
  line += csv_entry_domavg(m_cum.velocity.load(),
                           m_cum.robot_count,
                           true);
  return true;
} /* csv_line_build() */

void movement_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const metrics::fsm::movement_metrics&>(metrics);
  ++m_interval.robot_count;
  ++m_cum.robot_count;
  auto cum_dist = m_cum.distance.load();
  auto int_dist = m_interval.distance.load();
  auto cum_vel = m_cum.velocity.load();
  auto int_vel = m_interval.velocity.load();
  m_cum.distance.compare_exchange_strong(cum_dist,
                                               cum_dist + m.distance().v());
  m_interval.distance.compare_exchange_strong(int_dist,
                                               int_dist + m.distance().v());
  m_cum.velocity.compare_exchange_strong(cum_vel,
                                               cum_vel + m.velocity().length());
  m_interval.velocity.compare_exchange_strong(int_vel,
                                               int_vel + m.velocity().length());
} /* collect() */

void movement_metrics_collector::reset_after_interval(void) {
  m_interval.distance = 0.0;
  m_interval.velocity = 0.0;
  m_interval.robot_count = 0;
} /* reset_after_interval() */

NS_END(fsm, metrics, fordyca);
