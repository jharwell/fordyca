/**
 * @file execution_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TASKS_EXECUTION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_TASKS_EXECUTION_METRICS_COLLECTOR_HPP_

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
 * @class execution_metrics_collector
 * @ingroup metrics tasks
 *
 * @brief Collector for \ref execution_metrics.
 *
 * Collects metrics about tasks as they are executed. Metrics are written out at
 * the specified interval, or every timestep, depending.
 */
class execution_metrics_collector : public rcppsw::metrics::base_metrics_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param interval Collection interval.
   */
  execution_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  void reset_after_timestep(void) override;

  size_t n_collectors(void) const { return m_count_stats.n_collectors; }
  uint n_harvesters(void) const { return m_count_stats.n_harvesters; }
  uint n_generalists(void) const { return m_count_stats.n_generalists; }

 private:
  struct count_stats {
    uint n_collectors;
    uint n_harvesters;
    uint n_generalists;

    uint n_cum_collectors;
    uint n_cum_harvesters;
    uint n_cum_generalists;
  };
  struct interface_stats {
    uint cum_collector_delay;
    uint cum_harvester_delay;
  };
  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  struct count_stats m_count_stats;
  struct interface_stats m_int_stats;
};

NS_END(tasks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TASKS_EXECUTION_METRICS_COLLECTOR_HPP_ */
