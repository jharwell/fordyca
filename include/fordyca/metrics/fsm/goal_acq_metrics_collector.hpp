/**
 * @file goal_acq_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQ_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQ_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class goal_acq_metrics_collector
 * @ingroup fordyca metrics fsm
 *
 * @brief Collector for \ref goal_acq_metrics.
 *
 * Metrics are written out at the end of the specified interval.
 */
class goal_acq_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param interval The collection interval.
   */
  goal_acq_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void reset_after_interval(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;

 private:
  /**
   * @brief Container for holding collected statistics. Must be atomic so counts
   * are valid in parallel metric collection contexts.
   */
  struct stats {
    std::atomic_uint n_true_exploring_for_goal{0};
    std::atomic_uint n_false_exploring_for_goal{0};
    std::atomic_uint n_vectoring_to_goal{0};
    std::atomic_uint n_acquiring_goal{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQ_METRICS_COLLECTOR_HPP_ */
