/**
 * \file site_selection_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

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
