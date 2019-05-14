/**
 * @file location_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_LOCATION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_LOCATION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "fordyca/metrics/grid2D_avg_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class location_metrics_collector
 * @ingroup fordyca metrics caches
 *
 * @brief Collector for \ref location_metrics.
 */
class location_metrics_collector final : public grid2D_avg_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of the arena.
   */
  location_metrics_collector(const std::string& ofname,
                             uint interval,
                             const rmath::vector2u& dims) :
      grid2D_avg_metrics_collector(ofname, interval, dims) {}

  uint collect_cell(const rmetrics::base_metrics& metrics,
                    const rmath::vector2u& coord) const override;
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_LOCATION_METRICS_COLLECTOR_HPP_ */
