/**
 * \file d2_metrics_manager.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_METRICS_D2_D2_METRICS_MANAGER_HPP_
#define INCLUDE_FORDYCA_ARGOS_METRICS_D2_D2_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/argos/metrics/d1/d1_metrics_manager.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"
#include "fordyca/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, metrics, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d2_metrics_manager
 * \ingroup argos metrics d2
 *
 * \brief Aggregates and metrics collection for d2 foraging. That
 * includes everything from \ref d1_metrics_manager, and also:
 *
 * - TAB metrics (rooted at Harvester)
 * - TAB metrics (rooted at Collector)
 * - Cache site selection
 */
class d2_metrics_manager final : public d1::d1_metrics_manager,
                                 public rer::client<d2_metrics_manager> {
 public:
  d2_metrics_manager(const rmconfig::metrics_config* mconfig,
                     const cdconfig::grid2D_config* gconfig,
                     const fs::path& output_root,
                     size_t n_block_clusters);

  void task_start_cb(const cta::polled_task* task,
                     const cta::ds::bi_tab* tab);
  void task_finish_or_abort_cb(const cta::polled_task* task);

  /**
   * \brief Collect metrics from the d2 controller.
   */
  template<class Controller>
  void collect_from_controller(const Controller* c) {
    d1::d1_metrics_manager::collect_from_controller(c);

    const auto *task = dynamic_cast<const cta::polled_task*>(c->current_task());

    /* only Cache Starter implements these metrics */
    if (nullptr != task &&
        tasks::d2::foraging_task::kCacheStarterName == task->name()) {
      collect(fmspecs::caches::kSiteSelection.scoped, *task);
    }
  }
 private:
  void register_standard(const rmconfig::metrics_config* mconfig);
};

NS_END(d2, metrics, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_METRICS_D2_D2_METRICS_MANAGER_HPP_ */
