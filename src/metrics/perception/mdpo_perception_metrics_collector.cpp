/**
 * @file mdpo_perception_metrics_collector.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/metrics/perception/mdpo_perception_metrics_collector.hpp"
#include <numeric>

#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/metrics/perception/mdpo_perception_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_perception_metrics_collector::mdpo_perception_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval),
      m_int_states(fsm::cell2D_fsm::kST_MAX_STATES, 0),
      m_cum_states(fsm::cell2D_fsm::kST_MAX_STATES, 0) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> mdpo_perception_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
      "int_avg_ST_EMPTY_inaccuracies",
      "int_avg_ST_HAS_BLOCK_inaccuracies",
      "int_avg_ST_HAS_CACHE_inaccuracies",
      "cum_avg_ST_EMPTY_inaccuracies",
      "cum_avg_ST_HAS_BLOCK_inaccuracies",
      "cum_avg_ST_HAS_CACHE_inaccuracies",
      "int_avg_known_percentage",
      "int_avg_unknown_percentage",
      "int_avg_knowledge_ratio",
      "cum_avg_known_percentage",
      "cum_avg_unknown_percentage",
      "cum_avg_knowledge_ratio"
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void mdpo_perception_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool mdpo_perception_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += csv_entry_intavg(m_int_states[fsm::cell2D_fsm::kST_EMPTY]);
  line += csv_entry_intavg(m_int_states[fsm::cell2D_fsm::kST_HAS_BLOCK]);
  line += csv_entry_intavg(m_int_states[fsm::cell2D_fsm::kST_HAS_CACHE]);
  line += csv_entry_tsavg(m_cum_states[fsm::cell2D_fsm::kST_EMPTY]);
  line += csv_entry_tsavg(m_cum_states[fsm::cell2D_fsm::kST_HAS_BLOCK]);
  line += csv_entry_tsavg(m_cum_states[fsm::cell2D_fsm::kST_HAS_CACHE]);

  line += csv_entry_intavg(m_int_known_percent);
  line += csv_entry_intavg(m_int_unknown_percent);
  line +=
      std::to_string(m_int_known_percent / m_int_unknown_percent) + separator();
  line += csv_entry_tsavg(m_cum_known_percent);
  line += csv_entry_tsavg(m_cum_unknown_percent);
  line +=
      std::to_string(m_cum_known_percent / m_cum_unknown_percent) + separator();
  return true;
} /* csv_line_build() */

void mdpo_perception_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const mdpo_perception_metrics&>(metrics);
  m_int_states[fsm::cell2D_fsm::kST_EMPTY] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_EMPTY);
  m_int_states[fsm::cell2D_fsm::kST_HAS_BLOCK] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_HAS_BLOCK);
  m_int_states[fsm::cell2D_fsm::kST_HAS_CACHE] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_HAS_CACHE);
  m_cum_states[fsm::cell2D_fsm::kST_EMPTY] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_EMPTY);
  m_cum_states[fsm::cell2D_fsm::kST_HAS_BLOCK] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_HAS_BLOCK);
  m_cum_states[fsm::cell2D_fsm::kST_HAS_CACHE] +=
      m.cell_state_inaccuracies(fsm::cell2D_fsm::kST_HAS_CACHE);

  m_int_known_percent += m.known_percentage();
  m_int_unknown_percent += m.unknown_percentage();
  m_cum_known_percent += m.known_percentage();
  m_cum_unknown_percent += m.unknown_percentage();
  ++m_int_robots;
  ++m_cum_robots;
} /* collect() */

void mdpo_perception_metrics_collector::reset_after_interval(void) {
  m_int_states.assign(m_int_states.size(), 0);
  m_cum_states.assign(m_cum_states.size(), 0);
  m_int_known_percent = 0.0;
  m_int_unknown_percent = 0.0;
  m_int_robots = 0;
} /* reset_after_interval() */

NS_END(perception, metrics, fordyca);
