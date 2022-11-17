/**
 * \file manipulation_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

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
    ral::mt_init(&e.events, 0UL);
    ral::mt_init(&e.penalties, 0UL);
  } /* for(e..) */
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
