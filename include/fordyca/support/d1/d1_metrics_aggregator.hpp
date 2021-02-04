/**
 * \file d1_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D1_D1_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D1_D1_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/controller/metrics/manipulation_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/support/d0/d0_metrics_aggregator.hpp"
#include "fordyca/metrics/perception/dpo_perception_metrics.hpp"
#include "fordyca/metrics/perception/mdpo_perception_metrics.hpp"
#include "fordyca//controller/foraging_controller.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/controller/cognitive/foraging_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta::ds {
class bi_tab;
} /* namespace ds */

namespace cosm::arena::repr {
class arena_cache;
} /* namespace cosm::arena */

NS_START(fordyca);

namespace controller { namespace d1 { class bitd_mdpo_controller; }}
namespace support { class base_cache_manager; }
NS_START(support, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d1_metrics_aggregator
 * \ingroup support d1
 *
 * \brief Aggregates and metrics collection for d1 foraging. That
 * includes everything from \ref d0_metrics_aggregator, and also:
 *
 * - FSM cache acquisition metrics
 * - Cache utilization metrics
 * - Cache lifecycle metrics
 * - World model metrics
 * - Task execution metrics (per task)
 * - TAB metrics (rooted at generalist)
 */
class d1_metrics_aggregator : public d0::d0_metrics_aggregator,
                                  public rer::client<d1_metrics_aggregator> {
 public:
  d1_metrics_aggregator(const cmconfig::metrics_config* mconfig,
                            const cdconfig::grid2D_config* gconfig,
                            const std::string& output_root,
                            size_t n_block_clusters);

  /**
   * \brief Collect metrics from a finished or aborted task.
   *
   * This cannot be collected synchronously per-timestep with the rest of the
   * metrics from the controller, because by the time metric collecting occurs,
   * the executive has already allocated a new task, and there is not any way to
   * know if a robot's current task is the result of an abort/finish (and is
   * therefore newly allocated and SHOULD have metrics collected from it), or is
   * just running normally.
   *
   * Solution: hook into the executive callback queue in order to correctly
   * capture statistics.
   */
  void task_finish_or_abort_cb(const cta::polled_task* task);

  void task_start_cb(const cta::polled_task*, const cta::ds::bi_tab* tab);

    /**
   * \brief Collect metrics from the d1 controller.
   */
    template<class Controller>
    void collect_from_controller(const Controller* const controller) {
      base_metrics_aggregator::collect_from_controller(controller);
      collect_controller_common(controller);
      /*
       * Only controllers with MDPO perception provide these.
       */
      auto mdpo = dynamic_cast<const metrics::perception::mdpo_perception_metrics*>(
          controller->perception());
      if (nullptr != mdpo) {
        collect("perception::mdpo", *mdpo);
      }
      /*
       * Only controllers with DPO perception provide these.
       */
      auto dpo = dynamic_cast<const metrics::perception::dpo_perception_metrics*>(
          controller->perception());
      if (nullptr != dpo) {
        collect("perception::dpo", *dpo);
      }
    }

  /**
   * \brief Collect utilization metrics from a cache in the arena.
   */
  void collect_from_cache(const carepr::arena_cache* cache);

  /**
   * \brief Collect lifecycle metrics across all caches in the arena.
   */
  void collect_from_cache_manager(
      const support::base_cache_manager* manager);

 protected:
  /**
   * \brief Register all collectors which require the task decomposition graph
   * depth.
   *
   * This is a protected function so that derived classes aggregating metrics
   * from deeper decomposition graphs can overwrite the original version, and
   * thereby reduce code duplication.
   */
  void register_with_decomp_depth(const cmconfig::metrics_config* mconfig,
                                  size_t depth);
 private:
  template<typename Controller>
  void collect_controller_common(const Controller* const controller) {
    collect("fsm::movement", *controller);
    collect("blocks::manipulation", *controller->block_manip_recorder());

    auto task = dynamic_cast<const cta::polled_task*>(controller->current_task());
    if (nullptr == task) {
      return;
    }
    collect("fsm::interference_counts", *task->mechanism());
    collect("blocks::transporter", *task->mechanism());
    collect_if("fsm::interference_locs2D",
               *task->mechanism(),
               [&](const rmetrics::base_metrics& metrics) {
                 auto& m = dynamic_cast<const csmetrics::interference_metrics&>(metrics);
                 return m.exp_interference();
               });
    collect_if(
        "blocks::acq_counts",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal();
        });
    collect_if(
        "blocks::acq_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.goal_acquired();
        });

    /*
     * We count "false" explorations as part of gathering metrics on where
     * robots explore.
     */
    collect_if(
        "blocks::acq_explore_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.is_exploring_for_goal().is_exploring;
        });
    collect_if(
        "blocks::acq_vector_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekBLOCK == m.acquisition_goal() &&
              m.is_vectoring_to_goal();
        });

    collect_if(
        "caches::acq_counts",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal();
        });
    collect_if(
        "caches::acq_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.goal_acquired();
        });

    /*
     * We count "false" explorations as part of gathering metrics on where
     * robots explore.
     */
    collect_if(
        "caches::acq_explore_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.is_exploring_for_goal().is_exploring;
        });
    collect_if(
        "caches::acq_vector_locs2D",
        *task->mechanism(),
        [&](const rmetrics::base_metrics& metrics) {
          auto& m = dynamic_cast<const csmetrics::goal_acq_metrics&>(
              metrics);
          return fsm::foraging_acq_goal::ekEXISTING_CACHE == m.acquisition_goal() &&
              m.is_vectoring_to_goal();
        });
    collect("tasks::distribution", *controller);
  } /* collect_controller_common() */

  void register_standard(const cmconfig::metrics_config* mconfig);

  void register_with_arena_dims2D(const cmconfig::metrics_config* mconfig,
                                  const rmath::vector2z& dims);
};

NS_END(d1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D1_D1_METRICS_AGGREGATOR_HPP_ */
