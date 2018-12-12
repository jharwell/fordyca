/**
 * @file temporal_variance_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

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
 * @class temporal_variance_metrics_collector
 * @ingroup metrics blocks
 *
 * @brief Collector for \ref temporal_variance_metrics.
 *
 * Metrics are written out every timestep.
 */
class temporal_variance_metrics_collector
    : public rcppsw::metrics::base_metrics_collector,
      public visitor::visitable_any<temporal_variance_metrics_collector> {
 public:
  /**
   * @param ofname The output file name.
   */
  explicit temporal_variance_metrics_collector(const std::string& ofname);

  void collect(const rcppsw::metrics::base_metrics& metrics) override;

 private:
  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  // clang-format off
  double m_swarm_motion_throttle{0.0};
  double m_env_block_manip{0.0};
  double m_env_cache_usage{0.0};
  // clang-format on
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TEMPORAL_VARIANCE_METRICS_COLLECTOR_HPP_ */
