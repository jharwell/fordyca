/**
 * @file world_model_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class world_model_metrics_collector
 * @ingroup metrics blocks
 *
 * @brief Collector for \ref world_model_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class world_model_metrics_collector
    : public rcppsw::metrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  world_model_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  /* clang-format off */
  std::vector<uint>   m_stats;
  std::vector<double> m_known{};
  std::vector<double> m_unknown{};
  /* clang-format on */
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_COLLECTOR_HPP_ */
