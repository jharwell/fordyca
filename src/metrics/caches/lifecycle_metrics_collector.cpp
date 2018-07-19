/**
 * @file lifecycle_metrics_collector.cpp
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
#include "fordyca/metrics/caches/lifecycle_metrics_collector.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
lifecycle_metrics_collector::lifecycle_metrics_collector(const std::string& ofname,
                                                         uint interval)
    : base_metrics_collector(ofname, interval), m_stats() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string lifecycle_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  return base_metrics_collector::csv_header_build(header) +
      "avg_created" + separator() +
      "avg_depleted" + separator();
  // clang-format on
} /* csv_header_build() */

void lifecycle_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool lifecycle_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.n_created) + separator();
  line += std::to_string(m_stats.n_depleted) + separator();
  return true;
} /* csv_line_build() */

void lifecycle_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const lifecycle_metrics&>(metrics);
  m_stats.n_created += m.caches_created();
  m_stats.n_depleted += m.caches_depleted();
} /* collect() */

void lifecycle_metrics_collector::reset_after_interval(void) {
  m_stats.n_created = 0;
  m_stats.n_depleted = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
