/**
 * @file depth1_metrics_collector.cpp
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
#include "fordyca/metrics/collectors/fsm/depth1_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/depth1_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_metrics_collector::depth1_metrics_collector(const std::string ofname,
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
std::string depth1_metrics_collector::csv_header_build(
    const std::string &header) {
  // clang-format off
  if (collect_cum()) {
  return base_metric_collector::csv_header_build(header) +
      "n_acquiring_cache" + separator() +
      "n_cum_acquiring_cache" + separator() +
      "n_vectoring_to_cache" + separator() +
      "n_cum_vectoring_to_cache" + separator() +
      "n_exploring_for_cache" + separator() +
      "n_cum_exploring_for_cache" + separator() +
      "n_transporting_to_cache" + separator() +
      "n_cum_transporting_to_cache" + separator();
  } else {
  return base_metric_collector::csv_header_build(header) +
      "n_acquiring_cache" + separator() +
      "n_vectoring_to_cache" + separator() +
      "n_exploring_for_cache" + separator() +
      "n_transporting_to_cache" + separator();
  }
  // clang-format on
} /* csv_header_build() */

void depth1_metrics_collector::reset(void) {
  base_metric_collector::reset();
  reset_after_interval();
} /* reset() */

void depth1_metrics_collector::collect(
    const collectible_metrics::base_collectible_metrics &metrics) {
  auto &m =
      static_cast<const collectible_metrics::fsm::depth1_metrics &>(
          metrics);
  m_stats.n_exploring_for_cache += m.is_exploring_for_cache();
  m_stats.n_acquiring_cache += m.is_acquiring_cache();
  m_stats.n_vectoring_to_cache += m.is_vectoring_to_cache();
  m_stats.n_transporting_to_cache += m.is_transporting_to_cache();

  if (collect_cum()) {
    m_stats.n_cum_exploring_for_cache += m.is_exploring_for_cache();
    m_stats.n_cum_acquiring_cache += m.is_acquiring_cache();
    m_stats.n_cum_vectoring_to_cache += m.is_vectoring_to_cache();
    m_stats.n_cum_transporting_to_cache += m.is_transporting_to_cache();
  }
} /* collect() */

bool depth1_metrics_collector::csv_line_build(std::string &line) {
  if (collect_cum()) {
    line = std::to_string(m_stats.n_acquiring_cache) + separator() +
           std::to_string(m_stats.n_cum_acquiring_cache) + separator() +
           std::to_string(m_stats.n_vectoring_to_cache) + separator() +
           std::to_string(m_stats.n_cum_vectoring_to_cache) + separator() +
           std::to_string(m_stats.n_exploring_for_cache) + separator() +
           std::to_string(m_stats.n_cum_exploring_for_cache) + separator() +
           std::to_string(m_stats.n_transporting_to_cache) + separator() +
           std::to_string(m_stats.n_cum_transporting_to_cache) + separator();
  } else {
    line = std::to_string(m_stats.n_acquiring_cache) + separator() +
           std::to_string(m_stats.n_vectoring_to_cache) + separator() +
           std::to_string(m_stats.n_exploring_for_cache) + separator() +
           std::to_string(m_stats.n_transporting_to_cache) + separator();
  }
  return true;
} /* store_foraging_stats() */

void depth1_metrics_collector::reset_after_interval(void) {
  m_stats.n_cum_exploring_for_cache = 0;
  m_stats.n_cum_acquiring_cache = 0;
  m_stats.n_cum_vectoring_to_cache = 0;
  m_stats.n_cum_transporting_to_cache = 0;
} /* reset_after_interval() */

void depth1_metrics_collector::reset_after_timestep(void) {
  m_stats.n_exploring_for_cache = 0;
  m_stats.n_acquiring_cache = 0;
  m_stats.n_vectoring_to_cache = 0;
  m_stats.n_transporting_to_cache = 0;
} /* reset_after_timestep() */

NS_END(fsm, collectors, metrics, fordyca);
