/**
 * @file robot_occupancy_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/robot_occupancy_metrics_collector.hpp"
#include "fordyca/metrics/robot_occupancy_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2u> robot_occupancy_metrics_collector::collect_cells(
    const rmetrics::base_metrics& metrics) const {
  auto& m = dynamic_cast<const robot_occupancy_metrics&>(metrics);
  std::vector<rmath::vector2u> ret;
  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      auto coord = rmath::vector2u(i,j);
      if (m.has_robot(coord)) {
        ret.push_back(coord);
      }
    } /* for(j..) */
  }   /* for(i..) */
  return ret;
} /* collect_cells() */

NS_END(metrics, fordyca);
