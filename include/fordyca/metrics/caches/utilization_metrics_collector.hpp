/**
 * @file utilization_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class utilization_metrics_collector
 * @ingroup fordyca metrics caches
 *
 * @brief Collector for \ref utilization_metrics.
 *
 * Metrics are output at the specified interval.
 */
class utilization_metrics_collector : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname Output file name.
   * @param interval Collection interval.
   */
  utilization_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void reset_after_interval(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;

 private:
  /**
   * @brief All stats are cumulative within an interval.
   */
  struct stats {
    uint int_blocks{0};
    uint int_pickups{0};
    uint int_drops{0};
    uint int_cache_count{0};

    uint cum_blocks{0};
    uint cum_pickups{0};
    uint cum_drops{0};
    uint cum_cache_count{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  struct stats m_stats{};
  /* clang-format on */
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_ */
