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

#ifndef INCLUDE_METRICS_STATEFUL_METRICS_AGGREGATOR_HPP_
#define INCLUDE_METRICS_STATEFUL_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { namespace depth0 { class stateful_foraging_controller; }}

NS_START(support, depth0);

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

class stateful_metrics_aggregator : public stateless_metrics_aggregator {
 public:
  stateful_metrics_aggregator(std::shared_ptr<rcppsw::er::server> server,
                               const struct params::metrics_params* params,
                               const std::string& output_root);

  void collect_from_controller(
      const controller::depth0::stateful_foraging_controller* controller);
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_METRICS_STATEFUL_METRICS_AGGREGATOR_HPP_ */
