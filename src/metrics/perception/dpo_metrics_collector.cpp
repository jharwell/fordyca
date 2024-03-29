/**
 * \file dpo_metrics_collector.cpp
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
#include "fordyca/metrics/perception/dpo_metrics_collector.hpp"

#include <numeric>

#include "fordyca/metrics/perception/dpo_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_metrics_collector::dpo_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  const auto* m = dynamic_cast<const dpo_metrics*>(&metrics);

  /*
   * This can happen because we are trying to collect different types of
   * metrics from the perception subsystem, which can be of a number of
   * types. Only one of those types implements these metrics, so we have to
   * test here. This isn't any less efficient, because we had to cast to
   * convert anyway.
   */
  if (nullptr == m) {
    return;
  }

  ++m_data.interval.robot_count;
  ++m_data.cum.robot_count;

  m_data.interval.known_blocks += m->n_known_blocks();
  m_data.interval.known_caches += m->n_known_caches();

  m_data.cum.known_blocks += m->n_known_blocks();
  m_data.cum.known_caches += m->n_known_caches();

  ral::mt_accum(m_data.interval.block_density_sum, m->avg_block_density().v());
  ral::mt_accum(m_data.cum.block_density_sum, m->avg_block_density().v());
  ral::mt_accum(m_data.interval.cache_density_sum, m->avg_cache_density().v());
  ral::mt_accum(m_data.cum.block_density_sum, m->avg_cache_density().v());
} /* collect() */

void dpo_metrics_collector::reset_after_interval(void) {
  m_data.interval.robot_count = 0;
  m_data.interval.known_blocks = 0;
  m_data.interval.known_caches = 0;
  m_data.interval.block_density_sum = 0.0;
  m_data.interval.cache_density_sum = 0.0;
} /* reset_after_interval() */

NS_END(perception, metrics, fordyca);
