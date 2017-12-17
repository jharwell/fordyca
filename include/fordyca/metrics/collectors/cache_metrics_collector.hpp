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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_CACHE_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_CACHE_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/metrics/collectors/base_metric_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

namespace visitor = rcppsw::patterns::visitor;

NS_START(collectors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_metrics_collector
 *
 * @brief Collect for \ref cache_metrics.
 *
 * Metrics are not output every timestep, but only on timesteps in which a
 * cache-related event occurs (pickup from/drop in).
 */
class cache_metrics_collector : public base_metric_collector,
                                public visitor::visitable_any<cache_metrics_collector> {
 public:
  cache_metrics_collector(const std::string ofname) :
      base_metric_collector(ofname, false), m_new_data(false), m_stats() {}

  void reset(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;

 private:
  struct stats {
    size_t total_blocks;
    size_t total_pickups;
    size_t total_drops;
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;

  bool         m_new_data;
  struct stats m_stats;
};

NS_END(metrics, collectors, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_CACHE_METRICS_COLLECTOR_HPP_ */
