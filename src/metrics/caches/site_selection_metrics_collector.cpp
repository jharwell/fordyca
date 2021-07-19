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
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void site_selection_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const site_selection_metrics&>(metrics);
  nlopt::result res = m.nlopt_result();
  if (!m.site_select_exec()) {
    return;
  }

  if (m.site_select_success()) {
    ++m_data.interval.n_successes;
    m_data.interval.nlopt_stopval +=
        static_cast<uint>(nlopt::result::STOPVAL_REACHED == res);
    m_data.interval.nlopt_ftol +=
        static_cast<uint>(nlopt::result::FTOL_REACHED == res);
    m_data.interval.nlopt_xtol +=
        static_cast<uint>(nlopt::result::XTOL_REACHED == res);
    m_data.interval.nlopt_maxeval +=
        static_cast<uint>(nlopt::result::MAXEVAL_REACHED == res);

    ++m_data.cum.n_successes;
    m_data.cum.nlopt_stopval +=
        static_cast<uint>(nlopt::result::STOPVAL_REACHED == res);
    m_data.cum.nlopt_ftol +=
        static_cast<uint>(nlopt::result::FTOL_REACHED == res);
    m_data.cum.nlopt_xtol +=
        static_cast<uint>(nlopt::result::XTOL_REACHED == res);
    m_data.cum.nlopt_maxeval +=
        static_cast<uint>(nlopt::result::MAXEVAL_REACHED == res);
  } else {
    ++m_data.interval.n_fails;
    ++m_data.cum.n_fails;
  }
} /* collect() */

void site_selection_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_successes = 0;
  m_data.interval.n_fails = 0;
  m_data.interval.nlopt_stopval = 0;
  m_data.interval.nlopt_ftol = 0;
  m_data.interval.nlopt_xtol = 0;
  m_data.interval.nlopt_maxeval = 0;
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
