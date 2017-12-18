/**
 * @file block_metrics_collector.cpp
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
#include "fordyca/metrics/collectors/block_metrics_collector.hpp"
#include "fordyca/metrics/collectible_metrics/block_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_metrics_collector::block_metrics_collector(const std::string ofname,
                                                 uint collect_interval)
    : base_metric_collector(ofname, true), m_metrics() {
  use_interval(true);
  interval(collect_interval);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block_metrics_collector::csv_header_build(
    const std::string &header) {
  // clang-format off
  if (collect_cum()) {
    return base_metric_collector::csv_header_build(header) +
        "avg_carries" + separator() +
        "cum_collected" + separator();
  } else {
    return base_metric_collector::csv_header_build(header);
    "block_carries" + separator();
  }
  // clang-format on
} /* csv_header_build() */

void block_metrics_collector::reset(void) {
  base_metric_collector::reset();
  m_metrics = {0, 0};
} /* reset() */

bool block_metrics_collector::csv_line_build(std::string &line) {
  double avg_carries = 0;
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }

  if (m_metrics.cum_collected > 0) {
    avg_carries =
        static_cast<double>(m_metrics.cum_carries) / m_metrics.cum_collected;
  }
  line = std::to_string(avg_carries) + separator() +
         std::to_string(m_metrics.cum_collected) + separator();
  return true;
} /* csv_line_build() */

void block_metrics_collector::collect(
    const collectible_metrics::base_collectible_metrics &metrics) {
  auto &m = dynamic_cast<const collectible_metrics::block_metrics &>(metrics);
  m_metrics.cum_carries += m.n_carries();
  ++m_metrics.cum_collected;
} /* collect() */

void block_metrics_collector::reset_after_interval(void) {
  m_metrics.cum_carries = 0;
  m_metrics.cum_collected = 0;
} /* reset_after_interval() */

NS_END(collectors, metrics, fordyca);
