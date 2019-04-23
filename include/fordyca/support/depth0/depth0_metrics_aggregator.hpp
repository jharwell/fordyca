/**
 * @file depth0_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/metrics/base_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class depth0_metrics_aggregator
 * @ingroup fordyca support depth0
 *
 * @brief Aggregates and metrics metric collection for depth0 foraging. That
 * includes:
 *
 * - FSM distance/block acquisition metrics
 */

class depth0_metrics_aggregator : public metrics::base_metrics_aggregator,
                                  public rer::client<depth0_metrics_aggregator> {
 public:
  depth0_metrics_aggregator(const params::metrics_params* mparams,
                            const std::string& output_root);


  template<class T>
  void collect_from_controller(const T* controller);
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_METRICS_AGGREGATOR_HPP_ */
