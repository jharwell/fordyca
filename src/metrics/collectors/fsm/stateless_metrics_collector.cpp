/**
 * @file stateless_metrics_collector.cpp
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
#include "fordyca/metrics/collectors/fsm/stateless_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/stateless_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_metrics_collector::stateless_metrics_collector(const std::string& ofname,
                                                         bool collect_cum,
                                                         uint collect_interval)
    : base_metric_collector(ofname, collect_cum), m_stats() {
  if (collect_cum) {
    use_interval(true);
    interval(collect_interval);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string stateless_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  if (collect_cum()) {
    return base_metric_collector::csv_header_build(header) +
        "n_exploring_for_block"  + separator() +
        "n_cum_exploring_for_block"  + separator() +
        "n_avoiding_collision" + separator() +
        "n_cum_avoiding_collision" + separator() +
        "n_transporting_to_nest" + separator() +
        "n_cum_transporting_to_nest" + separator();
  }
  return base_metric_collector::csv_header_build(header) +
      "n_exploring_for_block"  + separator() +
      "n_avoiding_collision" + separator() +
      "n_transporting_to_nest" + separator();
  // clang-format on
} /* csv_header_build() */

void stateless_metrics_collector::reset(void) {
  base_metric_collector::reset();
  reset_after_interval();
} /* reset() */

void stateless_metrics_collector::collect(
    const collectible_metrics::base_collectible_metrics& metrics) {
  auto& m =
      static_cast<const collectible_metrics::fsm::stateless_metrics&>(metrics);
  m_stats.n_exploring_for_block += static_cast<uint>(m.is_exploring_for_block());
  m_stats.n_transporting_to_nest +=
      static_cast<uint>(m.is_transporting_to_nest());
  m_stats.n_avoiding_collision += static_cast<uint>(m.is_avoiding_collision());

  if (collect_cum()) {
    m_stats.n_cum_exploring_for_block +=
        static_cast<uint>(m.is_exploring_for_block());
    m_stats.n_cum_transporting_to_nest +=
        static_cast<uint>(m.is_transporting_to_nest());
    m_stats.n_cum_avoiding_collision +=
        static_cast<uint>(m.is_avoiding_collision());
  }
} /* collect() */

bool stateless_metrics_collector::csv_line_build(std::string& line) {
  if (collect_cum()) {
    line = std::to_string(m_stats.n_exploring_for_block) + separator() +
           std::to_string(m_stats.n_cum_exploring_for_block) + separator() +
           std::to_string(m_stats.n_avoiding_collision) + separator() +
           std::to_string(m_stats.n_cum_avoiding_collision) + separator() +
           std::to_string(m_stats.n_transporting_to_nest) + separator() +
           std::to_string(m_stats.n_cum_transporting_to_nest) + separator();
  } else {
    line = std::to_string(m_stats.n_exploring_for_block) + separator() +
           std::to_string(m_stats.n_avoiding_collision) + separator() +
           std::to_string(m_stats.n_transporting_to_nest) + separator();
  }
  return true;
} /* store_foraging_stats() */

void stateless_metrics_collector::reset_after_timestep(void) {
  m_stats.n_avoiding_collision = 0;
  m_stats.n_exploring_for_block = 0;
  m_stats.n_transporting_to_nest = 0;
} /* reset_after_timestep() */

void stateless_metrics_collector::reset_after_interval(void) {
  m_stats.n_cum_avoiding_collision = 0;
  m_stats.n_cum_exploring_for_block = 0;
  m_stats.n_cum_transporting_to_nest = 0;
} /* reset_after_interval() */

NS_END(fsm, collectors, metrics, fordyca);
