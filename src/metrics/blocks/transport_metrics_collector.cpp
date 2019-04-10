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
std::string transport_metrics_collector::csv_header_build(
    const std::string& header) {
  /* clang-format off */
  return base_metrics_collector::csv_header_build(header) +
      "cum_collected" + separator() +
      "cum_ramp_collected" + separator() +
      "cum_cube_collected" + separator() +
      "int_avg_collected" + separator() +
      "cum_avg_collected" + separator() +
      "int_avg_cube_collected" + separator() +
      "cum_avg_cube_collected" + separator() +
      "int_avg_ramp_collected" + separator() +
      "cum_avg_ramp_collected" + separator() +
      "int_avg_transporters" + separator() +
      "cum_avg_transporters" + separator() +
      "int_avg_transport_time" + separator() +
      "cum_avg_transport_time" + separator() +
      "int_avg_initial_wait_time" + separator() +
      "cum_avg_initial_wait_time" + separator();
  /* clang-format on */
} /* csv_header_build() */

void transport_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool transport_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.cum_collected) + separator();
  line += std::to_string(m_stats.cum_ramp_collected) + separator();
  line += std::to_string(m_stats.cum_cube_collected) + separator();

  line +=
      std::to_string(m_stats.int_collected / static_cast<double>(interval())) +
      separator();
  line += std::to_string(m_stats.cum_collected /
                         static_cast<double>(timestep() + 1)) +
          separator();

  line += std::to_string(m_stats.int_cube_collected /
                         static_cast<double>(interval())) +
          separator();
  line += std::to_string(m_stats.cum_cube_collected /
                         static_cast<double>(timestep() + 1)) +
          separator();

  line += std::to_string(m_stats.int_ramp_collected /
                         static_cast<double>(interval())) +
          separator();
  line += std::to_string(m_stats.cum_ramp_collected /
                         static_cast<double>(timestep() + 1)) +
          separator();

  line += (m_stats.int_collected > 0)
              ? std::to_string(m_stats.int_transporters /
                               static_cast<double>(m_stats.int_collected))
              : "0";
  line += separator();

  line += (m_stats.cum_collected > 0)
              ? std::to_string(m_stats.cum_transporters /
                               static_cast<double>(m_stats.cum_collected))
              : "0";
  line += separator();

  line += (m_stats.int_collected > 0)
              ? std::to_string(m_stats.int_transport_time /
                               static_cast<double>(m_stats.int_collected))
              : "0";
  line += separator();

  line += (m_stats.cum_collected > 0)
              ? std::to_string(m_stats.cum_transport_time /
                               static_cast<double>(m_stats.cum_collected))
              : "0";
  line += separator();
  line += (m_stats.int_collected > 0)
              ? std::to_string(m_stats.int_initial_wait_time /
                               static_cast<double>(m_stats.int_collected))
              : "0";
  line += separator();
  line += (m_stats.cum_collected > 0)
              ? std::to_string(m_stats.cum_initial_wait_time /
                               static_cast<double>(m_stats.cum_collected))
              : "0";
  line += separator();

  return true;
} /* csv_line_build() */

void transport_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const transport_metrics&>(metrics);
  ++m_stats.int_collected;
  m_stats.int_cube_collected +=
      (transport_metrics::block_type::kCube == m.type());
  m_stats.int_ramp_collected +=
      (transport_metrics::block_type::kRamp == m.type());

  ++m_stats.cum_collected;
  m_stats.cum_cube_collected +=
      (transport_metrics::block_type::kCube == m.type());
  m_stats.cum_ramp_collected +=
      (transport_metrics::block_type::kRamp == m.type());

  m_stats.int_transporters += m.total_transporters();
  m_stats.cum_transporters += m.total_transporters();

  m_stats.int_transport_time += m.total_transport_time();
  m_stats.cum_transport_time += m.total_transport_time();

  m_stats.int_initial_wait_time += m.initial_wait_time();
  m_stats.cum_initial_wait_time += m.initial_wait_time();
} /* collect() */

void transport_metrics_collector::reset_after_interval(void) {
  m_stats.int_collected = 0;
  m_stats.int_cube_collected = 0;
  m_stats.int_ramp_collected = 0;
  m_stats.int_transporters = 0;
  m_stats.int_transport_time = 0;
  m_stats.int_initial_wait_time = 0;
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
