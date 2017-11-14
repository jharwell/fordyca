/**
 * @file depth0_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_ROBOT_METRICS_DEPTH0_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_ROBOT_METRICS_DEPTH0_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/metrics/collectors/base_metric_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);
namespace collectible_metrics { namespace robot_metrics { class depth0_metrics; } }

NS_START(collectors, robot_metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class depth0_collector : public base_metric_collector {
 public:
  explicit depth0_collector(const std::string ofname) :
      base_metric_collector(ofname), m_stats() {}

  void reset(void) override;
  void collect(const collectible_metrics::robot_metrics::depth0_metrics& metrics);
  void reset_on_timestep(void) override;

 private:
  struct stats {
    size_t n_acquiring_block;
    size_t n_vectoring_to_block;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats;
};

NS_END(robot_metrics, collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_ROBOT_METRICS_DEPTH0_DEPTH0_COLLECTOR_HPP_ */
