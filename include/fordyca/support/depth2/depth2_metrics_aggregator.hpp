/**
 * @file depth2_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { namespace depth2 { class grp_mdpo_controller; }}

NS_START(support, depth2);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class depth2_metrics_aggregator
 * @ingroup support depth2
 *
 * @brief Aggregates and metrics collection for depth2 foraging. That
 * includes everything from \ref depth1_metrics_aggregator, and also:
 *
 * - TAB metrics (rooted at harvester)
 * - TAB metrics (rooted at collector)
 */
class depth2_metrics_aggregator : public depth1::depth1_metrics_aggregator,
                                  public er::client<depth2_metrics_aggregator> {
 public:
  depth2_metrics_aggregator(const params::metrics_params* mparams,
                            const std::string& output_root);

  void task_alloc_cb(const rta::polled_task* task,
                     const rta::bi_tab* tab);
  void task_finish_or_abort_cb(const rta::polled_task* task);

  /**
   * @brief Collect metrics from the depth2 controller.
   */
  template<class ControllerType>
  void collect_from_controller(const ControllerType* c) {
    depth1::depth1_metrics_aggregator::collect_from_controller(c);
  }
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_METRICS_AGGREGATOR_HPP_ */
