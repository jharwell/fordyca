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

#ifndef INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_COLLECTOR_HPP_

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
NS_START(fordyca, metrics, caches);
namespace rmath = rcppsw::math;
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class location_metrics_collector
 * @ingroup metrics caches
 *
 * @brief Collector for \ref location_metrics.
 *
 * Location metrics are somewhat unusual, because they output a large 2D array
 * into a .csv each time they are written out. As such, at the specified
 * collection interval they are written out, capturing the state of the
 * arena in terms of an accumulated desired quantity (i.e. metrics are always
 * written out as cumulative averages) representing proportions of which cells
 * in the arena contain/have contained a cache in the past.
 */
class location_metrics_collector : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of the arena.
   */
  location_metrics_collector(const std::string& ofname,
                             uint interval,
                             const rmath::vector2u& dims);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;

 private:
  std::string csv_header_build(const std::string&) override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  rcppsw::ds::grid2D<uint> m_stats;
  uint                     m_total{0};  // Total count of all caches across all timesteps
  /* clang-format on */
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_LOCATION_METRICS_COLLECTOR_HPP_ */
