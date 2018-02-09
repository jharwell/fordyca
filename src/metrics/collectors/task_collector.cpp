/**
 * @file task_collector.cpp
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
#include "fordyca/metrics/collectors/task_collector.hpp"
#include "fordyca/metrics/collectible_metrics/task_metrics.hpp"
#include "fordyca/tasks/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
task_collector::task_collector(const std::string& ofname,
                               bool collect_cum,
                               uint collect_interval)
    : base_metric_collector(ofname, collect_cum),
      m_count_stats(),
      m_int_stats() {
  if (collect_cum) {
    use_interval(true);
    interval(collect_interval);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string task_collector::csv_header_build(const std::string& header) {
  // clang-format off
  std::string line = base_metric_collector::csv_header_build(header);
  if (collect_cum()) {
    return line +
        "collector_avg_interface_delay" + separator() +
        "forager_avg_interface_delay" + separator() +
        "n_collectors"  + separator() +
        "n_cum_collectors"  + separator() +
        "n_foragers" + separator() +
        "n_cum_foragers" + separator() +
        "n_generalists" + separator() +
        "n_cum_generalists" + separator();

  } else {
    return line +
        "n_collectors"  + separator() +
        "n_foragers" + separator() +
        "n_generalists" + separator();
  }
  // clang-format on
} /* csv_header_build() */

void task_collector::reset(void) {
  base_metric_collector::reset();
  reset_after_interval();
  reset_after_timestep();
} /* reset() */

void task_collector::collect(
    const collectible_metrics::base_collectible_metrics& metrics) {
  auto& m = static_cast<const collectible_metrics::task_metrics&>(metrics);
  m_count_stats.n_collectors +=
      static_cast<uint>(m.task_name() == tasks::foraging_task::kCollectorName);
  m_count_stats.n_foragers +=
      static_cast<uint>(m.task_name() == tasks::foraging_task::kForagerName);
  m_count_stats.n_generalists +=
      static_cast<uint>(m.task_name() == tasks::foraging_task::kGeneralistName);

  if (collect_cum()) {
    if (m.at_task_interface()) {
      m_int_stats.cum_collector_delay +=
          static_cast<uint>(m.task_name() == tasks::foraging_task::kCollectorName);
      m_int_stats.cum_forager_delay +=
          static_cast<uint>(m.task_name() == tasks::foraging_task::kForagerName);
    }

    m_count_stats.n_cum_collectors +=
        static_cast<uint>(m.task_name() == tasks::foraging_task::kCollectorName);
    m_count_stats.n_cum_foragers +=
        static_cast<uint>(m.task_name() == tasks::foraging_task::kForagerName);
    m_count_stats.n_cum_generalists += static_cast<uint>(
        m.task_name() == tasks::foraging_task::kGeneralistName);
  }
} /* collect() */

bool task_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }
  if (collect_cum()) {
    double avg = m_int_stats.cum_collector_delay /
                 (m_count_stats.n_cum_collectors/static_cast<double>(interval()));
    line = std::to_string(avg) + separator();
    avg = m_int_stats.cum_forager_delay /
          (m_count_stats.n_cum_foragers/static_cast<double>(interval()));
    line +=
           std::to_string(avg) + separator() +
           std::to_string(m_count_stats.n_collectors) + separator() +
           std::to_string(m_count_stats.n_cum_collectors) + separator() +
           std::to_string(m_count_stats.n_foragers) + separator() +
           std::to_string(m_count_stats.n_cum_foragers) + separator() +
           std::to_string(m_count_stats.n_generalists) + separator() +
           std::to_string(m_count_stats.n_cum_generalists) + separator();
  } else {
    line = std::to_string(m_count_stats.n_collectors) + separator() +
           std::to_string(m_count_stats.n_foragers) + separator() +
           std::to_string(m_count_stats.n_generalists) + separator();
  }
  return true;
} /* store_foraging_stats() */

void task_collector::reset_after_timestep(void) {
  m_count_stats.n_collectors = 0;
  m_count_stats.n_foragers = 0;
  m_count_stats.n_generalists = 0;
} /* reset_after_timestep() */

void task_collector::reset_after_interval(void) {
  m_count_stats.n_cum_collectors = 0;
  m_count_stats.n_cum_foragers = 0;
  m_count_stats.n_cum_generalists = 0;
  m_int_stats = {0, 0};
} /* reset_after_interval() */

NS_END(collectors, metrics, fordyca);
