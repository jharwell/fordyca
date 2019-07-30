/**
 * @file transport_metrics_collector.cpp
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
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/metrics/blocks/transport_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
transport_metrics_collector::transport_metrics_collector(const std::string& ofname,
                                                         uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> transport_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "cum_collected",
    "cum_ramp_collected",
    "cum_cube_collected",
    "int_avg_collected",
    "cum_avg_collected",
    "int_avg_cube_collected",
    "cum_avg_cube_collected",
    "int_avg_ramp_collected",
    "cum_avg_ramp_collected",
    "int_avg_transporters",
    "cum_avg_transporters",
    "int_avg_transport_time",
    "cum_avg_transport_time",
    "int_avg_initial_wait_time",
    "cum_avg_initial_wait_time"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void transport_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool transport_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_cum.collected) + separator();
  line += std::to_string(m_cum.ramp_collected) + separator();
  line += std::to_string(m_cum.cube_collected) + separator();

  line += csv_entry_intavg(m_interval.collected);
  line += csv_entry_tsavg(m_cum.collected);

  line += csv_entry_intavg(m_interval.cube_collected);
  line += csv_entry_tsavg(m_cum.cube_collected);
  line += csv_entry_intavg(m_interval.ramp_collected);
  line += csv_entry_tsavg(m_cum.ramp_collected);
  line += csv_entry_domavg(m_interval.collected, m_interval.transporters);
  line += csv_entry_domavg(m_cum.collected, m_cum.transporters);

  line += csv_entry_domavg(m_interval.collected, m_interval.transport_time);
  line += csv_entry_domavg(m_cum.collected, m_cum.transport_time);

  line += csv_entry_domavg(m_interval.collected, m_interval.initial_wait_time);
  line += csv_entry_domavg(m_cum.collected, m_cum.initial_wait_time, true);

  return true;
} /* csv_line_build() */

void transport_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const transport_metrics&>(metrics);
  ++m_interval.collected;
  m_interval.cube_collected += (repr::block_type::ekCUBE == m.type());
  m_interval.ramp_collected += (repr::block_type::ekRAMP == m.type());

  ++m_cum.collected;
  m_cum.cube_collected += (repr::block_type::ekCUBE == m.type());
  m_cum.ramp_collected += (repr::block_type::ekRAMP == m.type());

  m_interval.transporters += m.total_transporters();
  m_cum.transporters += m.total_transporters();

  m_interval.transport_time += m.total_transport_time().v();
  m_cum.transport_time += m.total_transport_time().v();

  m_interval.initial_wait_time += m.initial_wait_time().v();
  m_cum.initial_wait_time += m.initial_wait_time().v();
} /* collect() */

void transport_metrics_collector::reset_after_interval(void) {
  m_interval.collected = 0;
  m_interval.cube_collected = 0;
  m_interval.ramp_collected = 0;
  m_interval.transporters = 0;
  m_interval.transport_time = 0;
  m_interval.initial_wait_time = 0;
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
