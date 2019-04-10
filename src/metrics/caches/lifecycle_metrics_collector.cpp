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
#include <numeric>

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
  /* clang-format off */
  return base_metrics_collector::csv_header_build(header) +
      "int_created" + separator() +
      "int_depleted" + separator() +
      "int_avg_created" + separator() +
      "int_avg_depleted" + separator() +
      "cum_avg_created" + separator() +
      "cum_avg_depleted" + separator() +
      "int_avg_depletion_age" + separator() +
      "cum_avg_depletion_age" + separator();
  /* clang-format on */
} /* csv_header_build() */

void lifecycle_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool lifecycle_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats.int_created) + separator();
  line += std::to_string(m_stats.int_depleted) + separator();
  line += std::to_string(m_stats.int_created / static_cast<double>(interval())) +
          separator();
  line +=
      std::to_string(m_stats.int_depleted / static_cast<double>(interval())) +
      separator();
  line +=
      std::to_string(m_stats.cum_created / static_cast<double>(timestep() + 1)) +
      separator();
  line += std::to_string(m_stats.cum_depleted /
                         static_cast<double>(timestep() + 1)) +
          separator();
  line += (m_stats.int_depleted > 0) ?
          std::to_string(m_stats.int_depletion_sum /
                         static_cast<double>(m_stats.int_depleted)): "0";
  line += separator();

  line += (m_stats.cum_depleted > 0) ?
          std::to_string(m_stats.cum_depletion_sum /
                         static_cast<double>(m_stats.cum_depleted)): "0";
  line += separator();
  return true;
} /* csv_line_build() */

void lifecycle_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const lifecycle_metrics&>(metrics);
  auto ages = m.cache_depletion_ages();
  auto sum = std::accumulate(ages.begin(), ages.end(), 0);

  m_stats.int_created += m.caches_created();
  m_stats.int_depleted += m.caches_depleted();
  m_stats.int_depletion_sum += sum;

  m_stats.cum_created += m.caches_created();
  m_stats.cum_depleted += m.caches_depleted();
  m_stats.cum_depletion_sum += sum;
} /* collect() */

void lifecycle_metrics_collector::reset_after_interval(void) {
  m_stats.int_created = 0;
  m_stats.int_depleted = 0;
  m_stats.int_depletion_sum = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
