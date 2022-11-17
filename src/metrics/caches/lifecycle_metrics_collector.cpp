/**
 * \file lifecycle_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void lifecycle_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  const auto& m = static_cast<const lifecycle_metrics&>(metrics);
  auto ages = m.cache_depletion_ages();
  auto sum = std::accumulate(ages.begin(), ages.end(), rtypes::timestep(0));

  m_data.interval.created += m.caches_created();
  m_data.interval.depleted += m.caches_depleted();
  m_data.interval.discarded += m.caches_discarded();
  m_data.interval.depletion_sum += sum;

  m_data.cum.created += m.caches_created();
  m_data.cum.depleted += m.caches_depleted();
  m_data.cum.discarded += m.caches_discarded();
  m_data.cum.depletion_sum += sum;
} /* collect() */

void lifecycle_metrics_collector::reset_after_interval(void) {
  m_data.interval.created = 0;
  m_data.interval.depleted = 0;
  m_data.interval.discarded = 0;
  m_data.interval.depletion_sum = rtypes::timestep(0);
} /* reset_after_interval() */

NS_END(caches, metrics, fordyca);
