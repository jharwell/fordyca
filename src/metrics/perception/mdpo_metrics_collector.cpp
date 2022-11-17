/**
 * \file mdpo_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/perception/mdpo_metrics_collector.hpp"

#include <numeric>

#include "fordyca/metrics/perception/mdpo_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_metrics_collector::mdpo_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  const auto* m = dynamic_cast<const mdpo_metrics*>(&metrics);

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

  m_data.interval.states[cfsm::cell2D_state::ekST_EMPTY] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_EMPTY);
  m_data.interval.states[cfsm::cell2D_state::ekST_HAS_BLOCK] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_HAS_BLOCK);
  m_data.interval.states[cfsm::cell2D_state::ekST_HAS_CACHE] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_HAS_CACHE);
  m_data.cum.states[cfsm::cell2D_state::ekST_EMPTY] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_EMPTY);
  m_data.cum.states[cfsm::cell2D_state::ekST_HAS_BLOCK] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_HAS_BLOCK);
  m_data.cum.states[cfsm::cell2D_state::ekST_HAS_CACHE] +=
      m->cell_state_inaccuracies(cfsm::cell2D_state::ekST_HAS_CACHE);

  ral::mt_accum(m_data.interval.known_percent, m->known_percentage());
  ral::mt_accum(m_data.interval.unknown_percent, m->unknown_percentage());
  ral::mt_accum(m_data.cum.known_percent, m->known_percentage());
  ral::mt_accum(m_data.cum.unknown_percent, m->unknown_percentage());

  ++m_data.interval.robots;
  ++m_data.cum.robots;
} /* collect() */

void mdpo_metrics_collector::reset_after_interval(void) {
  for (auto& state : m_data.interval.states) {
    ral::mt_init(&state, 0U);
  } /* for(state..) */

  m_data.interval.known_percent = 0.0;
  m_data.interval.unknown_percent = 0.0;
  m_data.interval.robots = 0;
} /* reset_after_interval() */

NS_END(perception, metrics, fordyca);
