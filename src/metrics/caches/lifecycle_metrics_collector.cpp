/**
 * \file lifecycle_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
lifecycle_metrics_collector::lifecycle_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> lifecycle_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_created",
    "int_depleted",
    "int_avg_created",
    "int_avg_depleted",
    "cum_avg_created",
    "cum_avg_depleted",
    "int_avg_depletion_age",
    "cum_avg_depletion_age"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void lifecycle_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> lifecycle_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += rcppsw::to_string(m_stats.int_created) + separator();
  line += rcppsw::to_string(m_stats.int_depleted) + separator();
  line += csv_entry_intavg(m_stats.int_created);
  line += csv_entry_intavg(m_stats.int_depleted);
  line += csv_entry_tsavg(m_stats.cum_created);
  line += csv_entry_tsavg(m_stats.cum_depleted);
  line += csv_entry_domavg(m_stats.int_depletion_sum.v(), m_stats.int_depleted);
  line += csv_entry_domavg(m_stats.cum_depletion_sum.v(),
                           m_stats.cum_depleted,
                           true);
  return boost::make_optional(line);
} /* csv_line_build() */

void lifecycle_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const lifecycle_metrics&>(metrics);
  auto ages = m.cache_depletion_ages();
  auto sum = std::accumulate(ages.begin(), ages.end(), rtypes::timestep(0));

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
  m_stats.int_depletion_sum = rtypes::timestep(0);
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
