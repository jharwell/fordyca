/**
 * @file stateful_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/crw_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation { class polled_task; }}
NS_START(fordyca);

namespace controller { namespace depth0 { class stateful_controller; }}

NS_START(support, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_metrics_aggregator
 * @ingroup support depth0
 *
 * @brief Aggregates and metrics metric collection for stateful foraging. That
 * includes:
 *
 * - FSM distance/block acquisition metrics
 */

class stateful_metrics_aggregator : public crw_metrics_aggregator,
                                    public er::client<stateful_metrics_aggregator> {
 public:
  stateful_metrics_aggregator(const struct params::metrics_params* params,
                              const std::string& output_root);

  /**
   * @brief Collect metrics from the stateful controller.
   */
  void collect_from_controller(
      const controller::depth0::stateful_controller* controller);
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_METRICS_AGGREGATOR_HPP_ */
