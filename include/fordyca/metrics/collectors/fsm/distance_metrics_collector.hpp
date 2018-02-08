/**
 * @file distance_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_DISTANCE_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_DISTANCE_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/metrics/collectors/base_metric_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

namespace collectible_metrics { namespace fsm { class distance_metrics; } }
namespace visitor = rcppsw::patterns::visitor;

NS_START(collectors, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class distance_metrics_collector
 * @ingroup metrics fsm
 *
 * @brief Collector for \ref distance_metrics.
 *
 * Metrics are written out every timestep.
 */
class distance_metrics_collector : public base_metric_collector,
                                   public visitor::visitable_any<distance_metrics_collector> {
 public:
  distance_metrics_collector(const std::string& ofname, size_t n_robots) :
      base_metric_collector(ofname, true), m_n_robots(n_robots), m_stats() {}

  void reset(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;

 private:
  struct robot_stats {
    double cum_distance;
  };

  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  // clang-format off
  size_t                          m_n_robots;
  std::vector<struct robot_stats> m_stats;
  // clang-format on
};

NS_END(fsm, collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_FSM_DISTANCE_METRICS_COLLECTOR_HPP_ */
