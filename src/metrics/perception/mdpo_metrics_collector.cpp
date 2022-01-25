/**
 * \file mdpo_metrics_collector.cpp
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
void mdpo_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {

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

  auto int_known_percent = m_data.interval.known_percent.load();
  auto int_unknown_percent = m_data.interval.unknown_percent.load();
  auto cum_known_percent = m_data.cum.known_percent.load();
  auto cum_unknown_percent = m_data.cum.unknown_percent.load();

  m_data.interval.known_percent.compare_exchange_strong(
      int_known_percent, int_known_percent + m->known_percentage());
  m_data.interval.unknown_percent.compare_exchange_strong(
      int_unknown_percent, int_unknown_percent + m->unknown_percentage());
  m_data.cum.known_percent.compare_exchange_strong(
      cum_known_percent, cum_known_percent + m->known_percentage());
  m_data.cum.unknown_percent.compare_exchange_strong(
      cum_unknown_percent, cum_unknown_percent + m->unknown_percentage());
  ++m_data.interval.robots;
  ++m_data.cum.robots;
} /* collect() */

void mdpo_metrics_collector::reset_after_interval(void) {
  for (auto& state : m_data.interval.states) {
    std::atomic_init(&state, 0U);
  } /* for(state..) */

  m_data.interval.known_percent = 0.0;
  m_data.interval.unknown_percent = 0.0;
  m_data.interval.robots = 0;
} /* reset_after_interval() */

NS_END(perception, metrics, fordyca);
