/**
 * @file swarm_pos2D_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_POS2D_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_POS2D_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "fordyca/metrics/spatial/grid2D_avg_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class swarm_pos2D_metrics_collector
 * @ingroup fordyca metrics spatial
 *
 * @brief Collector for \ref swarm_dist2D_metrics, robot positions.
 */
class swarm_pos2D_metrics_collector final : public grid2D_avg_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of arena.
   */
  swarm_pos2D_metrics_collector(const std::string& ofname,
                              uint interval,
                              const rmath::vector2u& dims)
      : grid2D_avg_metrics_collector(ofname, interval, dims) {}

  uint collect_cell(const rmetrics::base_metrics& metrics,
                    const rmath::vector2u& coord) const override;
};

NS_END(spatial, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_SPATIAL_SWARM_POS2D_METRICS_COLLECTOR_HPP_ */
