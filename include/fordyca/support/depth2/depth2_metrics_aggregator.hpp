/**
 * \file depth2_metrics_aggregator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class depth2_metrics_aggregator
 * \ingroup support depth2
 *
 * \brief Aggregates and metrics collection for depth2 foraging. That
 * includes everything from \ref depth1_metrics_aggregator, and also:
 *
 * - TAB metrics (rooted at Harvester)
 * - TAB metrics (rooted at Collector)
 * - Cache site selection
 */
class depth2_metrics_aggregator final : public depth1::depth1_metrics_aggregator,
                                  public rer::client<depth2_metrics_aggregator> {
 public:
  depth2_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                            const cdconfig::grid2D_config* gconfig,
                            const std::string& output_root);

  void task_start_cb(const cta::polled_task* task,
                     const cta::ds::bi_tab* tab);
  void task_finish_or_abort_cb(const cta::polled_task* task);

  /**
   * \brief Collect metrics from the depth2 controller.
   */
  template<class ControllerType>
  void collect_from_controller(const ControllerType* c) {
    depth1::depth1_metrics_aggregator::collect_from_controller(c);

    auto task = dynamic_cast<const cta::polled_task*>(c->current_task());

    /* only Cache Starter implements these metrics */
    if (nullptr != task &&
        tasks::depth2::foraging_task::kCacheStarterName == task->name()) {
      collect("caches::site_selection", *task);
    }
  }
 private:
  void register_standard(const cmconfig::metrics_config* mconfig);
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_METRICS_AGGREGATOR_HPP_ */
