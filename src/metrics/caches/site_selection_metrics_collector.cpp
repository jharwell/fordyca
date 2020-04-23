/**
 * \file site_selection_metrics_collector.cpp
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
#include "fordyca/metrics/caches/site_selection_metrics_collector.hpp"

#include "fordyca/metrics/caches/site_selection_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
site_selection_metrics_collector::site_selection_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> site_selection_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_n_successes",
    "int_n_fails",
    "int_nlopt_stopval",
    "int_nlopt_ftol",
    "int_nlopt_xtol",
    "int_nlopt_maxeval",
    "cum_n_successes",
    "cum_n_fails",
    "cum_nlopt_stopval",
    "cum_nlopt_ftol",
    "cum_nlopt_xtol",
    "cum_nlopt_maxeval",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void site_selection_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> site_selection_metrics_collector::csv_line_build(
    void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  line += csv_entry_intavg(m_stats.int_n_successes);
  line += csv_entry_intavg(m_stats.int_n_fails);
  line += csv_entry_intavg(m_stats.int_nlopt_stopval);
  line += csv_entry_intavg(m_stats.int_nlopt_ftol);
  line += csv_entry_intavg(m_stats.int_nlopt_xtol);
  line += csv_entry_intavg(m_stats.int_nlopt_maxeval);

  line += csv_entry_tsavg(m_stats.cum_n_successes);
  line += csv_entry_tsavg(m_stats.cum_n_fails);
  line += csv_entry_tsavg(m_stats.cum_nlopt_stopval);
  line += csv_entry_tsavg(m_stats.cum_nlopt_ftol);
  line += csv_entry_tsavg(m_stats.cum_nlopt_xtol);
  line += csv_entry_tsavg(m_stats.cum_nlopt_maxeval, true);

  return boost::make_optional(line);
} /* csv_line_build() */

void site_selection_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const site_selection_metrics&>(metrics);
  nlopt::result res = m.nlopt_result();
  if (!m.site_select_exec()) {
    return;
  }

  if (m.site_select_success()) {
    ++m_stats.int_n_successes;
    m_stats.int_nlopt_stopval += static_cast<uint>(nlopt::result::STOPVAL_REACHED == res);
    m_stats.int_nlopt_ftol += static_cast<uint>(nlopt::result::FTOL_REACHED == res);
    m_stats.int_nlopt_xtol += static_cast<uint>(nlopt::result::XTOL_REACHED == res);
    m_stats.int_nlopt_maxeval += static_cast<uint>(nlopt::result::MAXEVAL_REACHED == res);

    ++m_stats.cum_n_successes;
    m_stats.cum_nlopt_stopval += static_cast<uint>(nlopt::result::STOPVAL_REACHED == res);
    m_stats.cum_nlopt_ftol += static_cast<uint>(nlopt::result::FTOL_REACHED == res);
    m_stats.cum_nlopt_xtol += static_cast<uint>(nlopt::result::XTOL_REACHED == res);
    m_stats.cum_nlopt_maxeval += static_cast<uint>(nlopt::result::MAXEVAL_REACHED == res);
  } else {
    ++m_stats.int_n_fails;
    ++m_stats.cum_n_fails;
  }
} /* collect() */

void site_selection_metrics_collector::reset_after_interval(void) {
  m_stats.int_n_successes = 0;
  m_stats.int_n_fails = 0;
  m_stats.int_nlopt_stopval = 0;
  m_stats.int_nlopt_ftol = 0;
  m_stats.int_nlopt_xtol = 0;
  m_stats.int_nlopt_maxeval = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
