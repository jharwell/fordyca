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
 *
 * @brief Collector for \ref task_metrics.
 *
 * Metrics are written out every timestep.
 */
class task_collector : public base_metric_collector {
 public:
  task_collector(const std::string ofname,
                 bool collect_cum,
                 uint collect_interval);

  void reset(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;
  void reset_after_interval(void) override;
  void reset_after_timestep(void) override;

  size_t n_collectors(void) const { return m_stats.n_collectors; }
  size_t n_foragers(void) const { return m_stats.n_foragers; }
  size_t n_generalists(void) const { return m_stats.n_generalists; }

 private:
  struct stats {
    size_t n_collectors;
    size_t n_foragers;
    size_t n_generalists;
    size_t n_cum_collectors;
    size_t n_cum_foragers;
    size_t n_cum_generalists;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats;
};

NS_END(collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_TASK_COLLECTOR_HPP_ */
