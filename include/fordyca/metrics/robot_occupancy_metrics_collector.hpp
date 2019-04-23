/**
 * @file robot_occupancy_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_ROBOT_OCCUPANCY_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_ROBOT_OCCUPANCY_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "fordyca/metrics/grid2D_avg_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class robot_occupancy_metrics_collector
 * @ingroup fordyca metrics
 *
 * @brief Collector for \ref robot_occupancy_metrics.
 */
class robot_occupancy_metrics_collector : public grid2D_avg_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of robot_occupancy.
   */
  robot_occupancy_metrics_collector(const std::string& ofname,
                                    uint interval,
                                    const rmath::vector2u& dims)
      : grid2D_avg_metrics_collector(ofname, interval, dims) {}

  uint collect_cell(const rcppsw::metrics::base_metrics& metrics,
                    const rmath::vector2u& coord) const override;
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ROBOT_OCCUPANCY_METRICS_COLLECTOR_HPP_ */
