/**
 * @file collision_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class collision_metrics_collector
 * @ingroup metrics fsm
 *
 * @brief Collector for \ref collision_metrics.
 *
 * Metrics are written out after the specified interval.
 */
class collision_metrics_collector : public rcppsw::metrics::base_metrics_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param interval Collection interval.
   */
  collision_metrics_collector(const std::string& ofname,
                             uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct stats {
    uint int_n_in_avoidance;
    uint int_n_entered_avoidance;
    uint int_n_exited_avoidance;
    uint int_avoidance_duration;

    uint cum_n_in_avoidance;
    uint cum_n_entered_avoidance;
    uint cum_n_exited_avoidance;
    uint cum_avoidance_duration;
  };

  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  struct stats m_stats;
  /* clang-format on */
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_COLLISION_METRICS_COLLECTOR_HPP_ */
