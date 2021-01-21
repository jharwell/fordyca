/**
 * \file d2_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D2_D2_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D2_D2_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/support/d1/d1_metrics_aggregator.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d2_metrics_aggregator
 * \ingroup support d2
 *
 * \brief Aggregates and metrics collection for d2 foraging. That
 * includes everything from \ref d1_metrics_aggregator, and also:
 *
 * - TAB metrics (rooted at Harvester)
 * - TAB metrics (rooted at Collector)
 * - Cache site selection
 */
class d2_metrics_aggregator final : public d1::d1_metrics_aggregator,
                                  public rer::client<d2_metrics_aggregator> {
 public:
  d2_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                            const cdconfig::grid2D_config* gconfig,
                            const std::string& output_root,
                            size_t n_block_clusters);

  void task_start_cb(const cta::polled_task* task,
                     const cta::ds::bi_tab* tab);
  void task_finish_or_abort_cb(const cta::polled_task* task);

  /**
   * \brief Collect metrics from the d2 controller.
   */
  template<class Controller>
  void collect_from_controller(const Controller* c) {
    d1::d1_metrics_aggregator::collect_from_controller(c);

    auto task = dynamic_cast<const cta::polled_task*>(c->current_task());

    /* only Cache Starter implements these metrics */
    if (nullptr != task &&
        tasks::d2::foraging_task::kCacheStarterName == task->name()) {
      collect("caches::site_selection", *task);
    }
  }
 private:
  void register_standard(const cmconfig::metrics_config* mconfig);
};

NS_END(d2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D2_D2_METRICS_AGGREGATOR_HPP_ */
