/**
 * @file stateless_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/metrics/base_metrics_aggregator.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller { namespace depth0 { class stateless_controller; }}
namespace representation {
class base_block;
} /* namespace representation */

namespace ds {
class arena_map;
} /* namespace ds */

NS_START(support, depth0);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class stateless_metrics_aggregator
 * @ingroup support depth0
 *
 * @brief Aggregates and metrics metric collection for stateless foraging. That
 * includes:
 *
 * - FSM distance metrics
 * - Block acquisition metrics
 * - FSM collision metrics
 * - Block transport metrics
 * - Block manipulation metrics
 *
 * Metrics are *NOT* reset after collection--that is the caller's responsibility.
 */
class stateless_metrics_aggregator : public metrics::base_metrics_aggregator,
                                     public er::client<stateless_metrics_aggregator> {
 public:
  stateless_metrics_aggregator(const struct params::metrics_params* params,
                               const std::string& output_root);

  /**
   * @brief Collect metrics from the stateless controller synchronously (i.e. at
   * the end of each timestep).
   */
  void collect_from_controller(
      const controller::depth0::stateless_controller* controller);

  /**
   * @brief Collect metrics from a block right before it is dropped in the nest.
   */
  void collect_from_block(const representation::base_block* block);

  /**
   * @brief Collect metrics from the arena each timestep.
   */
  void collect_from_arena(const ds::arena_map* arena);
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_METRICS_AGGREGATOR_HPP_ */
