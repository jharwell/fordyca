/**
 * @file grid2D_avg_metrics_collector.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_GRID2D_AVG_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_GRID2D_AVG_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <vector>
#include <string>

#include "rcppsw/ds/grid2D.hpp"
#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);
namespace rmath = rcppsw::math;
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class grid2D_avg_metrics_collector
 * @ingroup metrics
 *
 * @brief Base class for collectors using a 2D grid to fill with counts of
 * SOMETHING, to be averaged over the entire simulation.
 */
class grid2D_avg_metrics_collector : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   * @param dims Dimensions of grid.
   */
  grid2D_avg_metrics_collector(const std::string& ofname,
                               uint interval,
                               const rmath::vector2u& dims);

  /**
   * @brief Return a list of coordinates in which the count for SOMETHING should
   * be incremented.
   */
  virtual std::vector<rmath::vector2u> collect_cells(
      const rmetrics::base_metrics& metrics) const = 0;

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;

  uint xsize(void) const { return m_stats.xsize(); }
  uint ysize(void) const { return m_stats.ysize(); }

 private:
  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  rcppsw::ds::grid2D<uint> m_stats;
  uint                     m_count{0};
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_GRID2D_AVG_METRICS_COLLECTOR_HPP_ */
