/**
 * \file manipulation_metrics_collector.cpp
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
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"

#include "cosm/controller/metrics/manipulation_metrics.hpp"

#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_collector::manipulation_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void manipulation_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const ccmetrics::manipulation_metrics&>(metrics);

  for (uint i = 0; i < block_manip_events::ekMAX_EVENTS; ++i) {
    m_data.interval[i].events += m.status(i);
    m_data.interval[i].penalties += m.penalty(i).v();

    m_data.cum[i].events += m.status(i);
    m_data.cum[i].penalties += m.penalty(i).v();
  } /* for(i..) */
} /* collect() */

void manipulation_metrics_collector::reset_after_interval(void) {
  for (auto& e : m_data.interval) {
    std::atomic_init(&e.events, 0UL);
    std::atomic_init(&e.penalties, 0UL);
  } /* for(e..) */
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
