/**
 * @file block_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTORS_BLOCK_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTORS_BLOCK_METRICS_COLLECTOR_HPP_

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
 * @class block_metrics_collector
 *
 * @brief Collector for \ref block_metrics.
 *
 * Metrics are not written out until at least 1 block has been collected.
 */
class block_metrics_collector : public base_metric_collector,
                                public visitor::visitable_any<block_metrics_collector> {
 public:
  block_metrics_collector(const std::string ofname,
                          bool collect_cum,
                          uint collect_interval);

  void reset(void) override;
  void reset_after_interval(void) override;
  void reset_after_timestep(void) override;
  void collect(const collectible_metrics::base_collectible_metrics& metrics) override;
  size_t cum_collected(void) const { return m_metrics.cum_collected; }

 private:
  struct block_metrics {
    size_t cum_collected; /* aggregate across blocks, not reset each timestep*/
    size_t cum_carries; /* aggregate across blocks, not reset each timstep */
    size_t block_carries; /* carries for most recently collected block */
  };

  std::string csv_header_build(const std::string& header = "") override;
  bool csv_line_build(std::string& line) override;
  struct block_metrics m_metrics;
};

NS_END(collectors, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTORS_BLOCK_METRICS_COLLECTOR_HPP_ */
