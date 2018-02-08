/**
 * @file task_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_TASK_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_TASK_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "fordyca/metrics/collectors/base_metric_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class task_collector
 * @ingroup metrics
 *
 * @brief Collector for \ref task_metrics.
 *
 * Metrics are written out at the specified interval, or every timestep,
 * depending.
 */
class task_collector : public base_metric_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param collect_cum If \c TRUE, then metrics will be accumulated during the
   * specified interval, and written out and reset at the end of it. If
   * \c FALSE, they will be written out every timestep.
   * @param collect_interval The interval. Ignored if collect_cum is \c FALSE.
   * @param n_robots # of robots in the swarm.
   */
  task_collector(const std::string& ofname,
                 bool collect_cum,
                 uint collect_interval,
                 size_t n_robots);

  void reset(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;
  void reset_after_interval(void) override;
  void reset_after_timestep(void) override;

  size_t n_collectors(void) const { return m_count_stats.n_collectors; }
  size_t n_foragers(void) const { return m_count_stats.n_foragers; }
  size_t n_generalists(void) const { return m_count_stats.n_generalists; }

 private:
  struct task_count_stats {
    size_t n_collectors;
    size_t n_foragers;
    size_t n_generalists;
    size_t n_cum_collectors;
    size_t n_cum_foragers;
    size_t n_cum_generalists;
  };
  struct task_interface_stats {
    double cum_interface_time;
  };
  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  size_t m_n_robots;
  struct task_count_stats m_count_stats;
  std::vector<struct task_interface_stats> m_int_stats;
};

NS_END(collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_TASK_COLLECTOR_HPP_ */
