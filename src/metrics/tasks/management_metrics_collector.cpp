/**
 * @file management_metrics_collector.cpp
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
#include "fordyca/metrics/tasks/management_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/management_metrics.hpp"
#include "fordyca/tasks/foraging_task.hpp"
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
management_metrics_collector::management_metrics_collector(const std::string& ofname,
                               bool collect_cum,
                               uint collect_interval)
    : base_metrics_collector(ofname, collect_cum),
      m_sel_stats(),
      m_partition_stats(),
      m_alloc_stats() {
  if (collect_cum) {
    use_interval(true);
    interval(collect_interval);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string management_metrics_collector::csv_header_build(const std::string& header) {
  // clang-format off
  std::string line = base_metrics_collector::csv_header_build(header);
    return line +
        "forager_subtask_count" + separator() +
        "collector_subtask_count" + separator() +
        "partition_count"  + separator() +
        "no_partition_count"  + separator() +
        "alloc_sw_count" + separator() +
        "task_abort_count" + separator();
  // clang-format on
} /* csv_header_build() */

void management_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void management_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = static_cast<const task_metrics::management_metrics&>(metrics);
  if (m.has_new_allocation()) {
    if (m.employed_partitioning()) {
      ++m_partition_stats.n_partition;
      m_sel_stats.n_collectors +=
          static_cast<uint>(m.current_task_name() == tasks::foraging_task::kCollectorName);
      m_sel_stats.n_foragers +=
          static_cast<uint>(m.current_task_name() == tasks::foraging_task::kForagerName);
    } else {
      ++m_partition_stats.n_no_partition;
    }
    m_alloc_stats.n_alloc_sw += static_cast<uint>(m.has_changed_allocation());
  }
  m_alloc_stats.n_abort += static_cast<uint>(m.has_aborted_task());
} /* collect() */

bool management_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }

  line += std::to_string(m_sel_stats.n_foragers) + separator() +
      std::to_string(m_sel_stats.n_collectors) + separator() +
      std::to_string(m_partition_stats.n_partition) + separator() +
      std::to_string(m_partition_stats.n_no_partition) + separator() +
      std::to_string(m_alloc_stats.n_alloc_sw) + separator() +
      std::to_string(m_alloc_stats.n_abort) + separator();
  return true;
} /* store_foraging_stats() */

void management_metrics_collector::reset_after_interval(void) {
  m_sel_stats = {0, 0};
  m_partition_stats = {0, 0};
  m_alloc_stats = {0, 0};
} /* reset_after_interval() */

NS_END(metrics, fordyca, tasks);
