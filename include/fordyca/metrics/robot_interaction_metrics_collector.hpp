/**
 * @file robot_interaction_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/swarm/interactivity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class robot_interaction_metrics_collector
 * @ingroup metrics
 *
 * @brief Collector for \ref robot_interaction_metrics.
 *
 * Metrics are written out each timestep.
 */
class robot_interaction_metrics_collector
    : public rcppsw::metrics::base_metrics_collector,
      public visitor::visitable_any<robot_interaction_metrics_collector> {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  robot_interaction_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;

 private:
  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  // clang-format off
  rcppsw::swarm::interactivity m_cum{};
  std::vector<double>          m_cum_stats{};
  // clang-format on
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ROBOT_INTERACTION_METRICS_COLLECTOR_HPP_ */
