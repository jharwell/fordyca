/**
 * @file world_model_metrics_collector.cpp
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
#include "fordyca/metrics/world_model_metrics_collector.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
world_model_metrics_collector::world_model_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval),
      m_stats(fsm::cell2D_fsm::ST_MAX_STATES, 0) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string world_model_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  return base_metrics_collector::csv_header_build(header) +
      "ST_EMPTY_inaccuracies" + separator() +
      "ST_HAS_BLOCK_inaccuracies" + separator() +
      "ST_HAS_CACHE_inaccuracies" + separator();
  // clang-format on
} /* csv_header_build() */

void world_model_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool world_model_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  line += std::to_string(m_stats[fsm::cell2D_fsm::ST_EMPTY]) + separator();
  line += std::to_string(m_stats[fsm::cell2D_fsm::ST_HAS_BLOCK]) + separator();
  line += std::to_string(m_stats[fsm::cell2D_fsm::ST_HAS_CACHE]) + separator();
  return true;
} /* csv_line_build() */

void world_model_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const world_model_metrics&>(metrics);
  m_stats[fsm::cell2D_fsm::ST_UNKNOWN] += m.cell_state_inaccuracies(fsm::cell2D_fsm::ST_UNKNOWN);
  m_stats[fsm::cell2D_fsm::ST_EMPTY] += m.cell_state_inaccuracies(fsm::cell2D_fsm::ST_EMPTY);
  m_stats[fsm::cell2D_fsm::ST_HAS_BLOCK] += m.cell_state_inaccuracies(fsm::cell2D_fsm::ST_HAS_BLOCK);
  m_stats[fsm::cell2D_fsm::ST_HAS_CACHE] += m.cell_state_inaccuracies(fsm::cell2D_fsm::ST_HAS_CACHE);
} /* collect() */

void world_model_metrics_collector::reset_after_interval(void) {
  m_stats.assign(m_stats.size(), 0);
} /* reset_after_interval() */

NS_END(metrics, fordyca);
