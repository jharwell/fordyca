/**
 * @file base_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/metrics/collector_group.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct metrics_params;
}

NS_START(metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_metrics_aggregator
 * @ingroup metrics
 *
 * @brief Base class for aggregating collection of metrics for various
 * sources. Extends \ref rcppsw::metrics::collector_group to include
 * initialization bits to make loop functions simpler/clearer.
 */
class base_metrics_aggregator
    : public rcppsw::er::client<base_metrics_aggregator>,
      public rcppsw::metrics::collector_group {
 public:
  base_metrics_aggregator(const struct params::metrics_params* params,
                          const std::string& output_root);
  virtual ~base_metrics_aggregator(void) = default;

 protected:
  const std::string& metrics_path(void) const { return m_metrics_path; }

 private:
  // clang-format off
  std::string m_metrics_path{""};
  // clang-format on
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BASE_METRICS_AGGREGATOR_HPP_ */
