/**
 * @file management_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TASKS_MANAGEMENT_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_TASKS_MANAGEMENT_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tasks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class management_metrics_collector
 * @ingroup metrics
 *
 * @brief Collector for \ref metrics.
 *
 * Collects metrics about the allocation of tasks at the level of the task
 * executive/controller. Metrics are written out at the specified interval, or
 * every timestep, depending.
 */
class management_metrics_collector : public rcppsw::metrics::base_metrics_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param interval Collection interval.
   */
  management_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct subtask_selection_stats {
    uint n_harvesters;
    uint n_collectors;
  };

  struct partitioning_stats {
    uint n_partition;
    uint n_no_partition;
  };

  struct allocation_stats {
    uint n_alloc_sw;
    uint n_abort;
  };

  struct finish_stats {
    uint n_completed;
    uint n_harvester_completed;
    uint n_collector_completed;
    uint n_generalist_completed;

    uint cum_collector_exec_time;
    uint cum_harvester_exec_time;
    uint cum_generalist_exec_time;
    uint cum_task_exec_time;
  };

  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  struct subtask_selection_stats m_sel_stats;
  struct partitioning_stats m_partition_stats;
  struct allocation_stats m_alloc_stats;
  struct finish_stats m_finish_stats;
};

NS_END(tasks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TASKS_MANAGEMENT_METRICS_COLLECTOR_HPP_ */
