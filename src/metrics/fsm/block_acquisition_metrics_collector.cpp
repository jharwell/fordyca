/**
 * @file block_acquisition_metrics_collector.cpp
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
#include "fordyca/metrics/fsm/block_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/fsm/block_acquisition_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_acquisition_metrics_collector::block_acquisition_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval), m_stats() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block_acquisition_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  return base_metrics_collector::csv_header_build(header) +
      "n_acquiring_block"  + separator() +
      "n_cum_acquiring_block"  + separator() +
      "n_exploring_for_block"  + separator() +
      "n_cum_exploring_for_block"  + separator() +
      "n_vectoring_to_block"  + separator() +
      "n_cum_vectoring_to_block"  + separator() +
      "n_avoiding_collision" + separator() +
      "n_cum_avoiding_collision" + separator();
  // clang-format on
} /* csv_header_build() */

void block_acquisition_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void block_acquisition_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const metrics::fsm::block_acquisition_metrics&>(metrics);
  m_stats.n_acquiring_block += static_cast<uint>(m.is_acquiring_block());
  m_stats.n_vectoring_to_block += static_cast<uint>(m.is_vectoring_to_block());
  m_stats.n_exploring_for_block += static_cast<uint>(m.is_vectoring_to_block());
  m_stats.n_avoiding_collision += static_cast<uint>(m.is_avoiding_collision());

  m_stats.n_cum_exploring_for_block += static_cast<uint>(m.is_acquiring_block());
  m_stats.n_cum_acquiring_block += static_cast<uint>(m.is_acquiring_block());
  m_stats.n_cum_vectoring_to_block +=
      static_cast<uint>(m.is_vectoring_to_block());
  m_stats.n_cum_avoiding_collision +=
      static_cast<uint>(m.is_avoiding_collision());
} /* collect() */

bool block_acquisition_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line = std::to_string(m_stats.n_acquiring_block) + separator() +
         std::to_string(m_stats.n_cum_acquiring_block) + separator() +
         std::to_string(m_stats.n_vectoring_to_block) + separator() +
         std::to_string(m_stats.n_cum_vectoring_to_block) + separator() +
         std::to_string(m_stats.n_exploring_for_block) + separator() +
         std::to_string(m_stats.n_cum_exploring_for_block) + separator() +
         std::to_string(m_stats.n_avoiding_collision) + separator() +
         std::to_string(m_stats.n_cum_avoiding_collision) + separator();
  return true;
} /* csv_line_build() */

void block_acquisition_metrics_collector::reset_after_interval(void) {
  m_stats.n_cum_acquiring_block = 0;
  m_stats.n_cum_vectoring_to_block = 0;
  m_stats.n_cum_exploring_for_block = 0;
  m_stats.n_cum_avoiding_collision = 0;
} /* reset_after_interval() */

void block_acquisition_metrics_collector::reset_after_timestep(void) {
  m_stats.n_acquiring_block = 0;
  m_stats.n_vectoring_to_block = 0;
  m_stats.n_exploring_for_block = 0;
  m_stats.n_avoiding_collision = 0;
} /* reset_after_timestep() */

NS_END(fsm, metrics, fordyca);
