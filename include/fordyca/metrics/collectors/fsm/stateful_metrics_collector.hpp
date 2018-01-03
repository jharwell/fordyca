/**
 * @file stateful_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_STATEFUL_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_STATEFUL_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/metrics/collectors/base_metric_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectors, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_metrics_collector
 * @ingroup metrics fsm
 *
 * @brief Collector for \ref stateful_metrics.
 *
 * Metrics are written out every timestep, or after the specified interval,
 * depending.
 */
class stateful_metrics_collector : public base_metric_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param collect_cum If \c TRUE, then metrics will be accumulated during the
   * specified interval, and written out and reset at the end of it. If
   * \c FALSE, they will be written out every timestep.
   * @param collect_interval The interval. Ignored if collect_cum is \c FALSE.
   */
  stateful_metrics_collector(const std::string ofname,
                             bool collect_cum,
                             uint collect_interval);

  void reset(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;
  void reset_after_interval(void) override;
  void reset_after_timestep(void) override;

 private:
  struct stats {
    size_t n_acquiring_block;
    size_t n_vectoring_to_block;

    size_t n_cum_acquiring_block;
    size_t n_cum_vectoring_to_block;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats;
};

NS_END(fsm, collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_STATEFUL_METRICS_COLLECTOR_HPP_ */
