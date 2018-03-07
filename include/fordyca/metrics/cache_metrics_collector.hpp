/**
 * @file cache_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHE_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHE_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <set>
#include <string>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_metrics_collector
 * @ingroup metrics
 *
 * @brief Collector for \ref cache_metrics.
 *
 * Metrics are output at the specified interval.
 */
class cache_metrics_collector
    : public rcppsw::metrics::base_metrics_collector,
      public visitor::visitable_any<cache_metrics_collector> {
 public:

  /**
   * @param ofname Output file name.
   * @param interval Collection interval.
   */
  cache_metrics_collector(const std::string& ofname,
                          uint interval);

  void reset(void) override;
  void reset_after_interval(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;

 private:
  /**
   * @brief All stats are cumulative within an interval.
   */
  struct stats {
    uint n_blocks;
    uint n_pickups;
    uint n_drops;
    uint n_penalty_steps;
  };

  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  // clang-format off
  uint           m_penalty_count{0};
  struct stats   m_stats;

  /**
   * IDs of the caches that had events in the current interval, for use in
   * averaging statistics.
   */
  std::set<uint> m_cache_ids;
  // clang-format on
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHE_METRICS_COLLECTOR_HPP_ */
