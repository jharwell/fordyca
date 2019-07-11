/**
 * @file movement_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/types/spatial_dist.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class movement_metrics_collector
 * @ingroup fordyca metrics fsm
 *
 * @brief Collector for \ref movement_metrics.
 *
 * Metrics are written out every timestep.
 */
class movement_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  movement_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct stats {
    /* clang-format off */
    rtypes::spatial_dist int_distance{0.0};
    uint                 int_robot_count{0};
    double               int_velocity{0.0};

    rtypes::spatial_dist cum_distance{0.0};
    uint                 cum_robot_count{0};
    double               cum_velocity{0.0};
    /* clang-format on */
  };

  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  struct stats m_stats{};
  /* clang-format on */
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_MOVEMENT_METRICS_COLLECTOR_HPP_ */
