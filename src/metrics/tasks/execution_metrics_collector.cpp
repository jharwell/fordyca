/**
 * @file execution_metrics_collector.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/metrics/tasks/execution_metrics_collector.hpp"
#include "fordyca/tasks/foraging_task.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/task_allocation/logical_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tasks);
namespace task_metrics = rcppsw::metrics::tasks;
namespace tasks = fordyca::tasks;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
execution_metrics_collector::execution_metrics_collector(const std::string& ofname,
                                                         uint interval)
    : base_metrics_collector(ofname, interval), m_count_stats(), m_int_stats() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string execution_metrics_collector::csv_header_build(
    const std::string& header) {
  // clang-format off
  std::string line = base_metrics_collector::csv_header_build(header);
  return line +
      "collector_avg_interface_delay" + separator() +
      "harvester_avg_interface_delay" + separator() +
      "n_collectors"  + separator() +
      "n_cum_collectors"  + separator() +
      "n_harvesters" + separator() +
      "n_cum_harvesters" + separator() +
      "n_generalists" + separator() +
      "n_cum_generalists" + separator();
  // clang-format on
} /* csv_header_build() */

void execution_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
  reset_after_timestep();
} /* reset() */

void execution_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const task_metrics::execution_metrics&>(metrics);
  auto& task =
      dynamic_cast<const rcppsw::task_allocation::logical_task&>(metrics);

  m_count_stats.n_collectors +=
      static_cast<uint>(task.name() == tasks::foraging_task::kCollectorName);
  m_count_stats.n_harvesters +=
      static_cast<uint>(task.name() == tasks::foraging_task::kHarvesterName);
  m_count_stats.n_generalists +=
      static_cast<uint>(task.name() == tasks::foraging_task::kGeneralistName);

  if (m.at_interface()) {
    m_int_stats.cum_collector_delay +=
        static_cast<uint>(task.name() == tasks::foraging_task::kCollectorName);
    m_int_stats.cum_harvester_delay +=
        static_cast<uint>(task.name() == tasks::foraging_task::kHarvesterName);
  }

  m_count_stats.n_cum_collectors +=
      static_cast<uint>(task.name() == tasks::foraging_task::kCollectorName);
  m_count_stats.n_cum_harvesters +=
      static_cast<uint>(task.name() == tasks::foraging_task::kHarvesterName);
  m_count_stats.n_cum_generalists += static_cast<uint>(
      task.name() == fordyca::tasks::foraging_task::kGeneralistName);
} /* collect() */

bool execution_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  double avg =
      m_int_stats.cum_collector_delay /
      (m_count_stats.n_cum_collectors / static_cast<double>(interval()));
  line = std::to_string(avg) + separator();
  avg = m_int_stats.cum_harvester_delay /
        (m_count_stats.n_cum_harvesters / static_cast<double>(interval()));

  line += std::to_string(avg) + separator() +
          std::to_string(m_count_stats.n_collectors) + separator() +
          std::to_string(m_count_stats.n_cum_collectors) + separator() +
          std::to_string(m_count_stats.n_harvesters) + separator() +
          std::to_string(m_count_stats.n_cum_harvesters) + separator() +
          std::to_string(m_count_stats.n_generalists) + separator() +
          std::to_string(m_count_stats.n_cum_generalists) + separator();
  return true;
} /* store_foraging_stats() */

void execution_metrics_collector::reset_after_timestep(void) {
  m_count_stats.n_collectors = 0;
  m_count_stats.n_harvesters = 0;
  m_count_stats.n_generalists = 0;
} /* reset_after_timestep() */

void execution_metrics_collector::reset_after_interval(void) {
  m_count_stats.n_cum_collectors = 0;
  m_count_stats.n_cum_harvesters = 0;
  m_count_stats.n_cum_generalists = 0;
  m_int_stats = {0, 0};
} /* reset_after_interval() */

NS_END(metrics, fordyca, tasks);
