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
#include <string>
#include <vector>

#include "rcppsw/ds/grid2D.hpp"
#include "rcppsw/metrics/base_metrics_collector.hpp"

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
 * @ingroup metrics blocks
 *
 * @brief Collector for \ref robot_occupancy_metrics.
 *
 * Robot_Occupancy metrics are somewhat unusual, because they output a large 2D
 * array into a .csv each time they are written out. As such, at the specified
 * collection interval the are written out, capturing the state of the
 * robot_occupancy for in terms of an accumulated desired quantity (i.e. metrics
 * are always written out as cumulative averages).
 */
class robot_occupancy_metrics_collector
    : public rcppsw::metrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of robot_occupancy.
   */
  robot_occupancy_metrics_collector(const std::string& ofname,
                                    uint interval,
                                    const rmath::vector2u& dims);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;

 private:
  std::string csv_header_build(const std::string&) override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  rcppsw::ds::grid2D<uint> m_stats;
  uint                     m_total{0};  // Total count of all robots across all timesteps
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ROBOT_OCCUPANCY_METRICS_COLLECTOR_HPP_ */
