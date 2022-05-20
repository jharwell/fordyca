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
