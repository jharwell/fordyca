/**
 * @file depth1_metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/support/depth0/stateful_metrics_aggregator.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"
#include "fordyca/metrics/world_model_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class bi_tab;
}}
namespace ta = rcppsw::task_allocation;
NS_START(fordyca);

namespace controller { namespace depth1 { class greedy_partitioning_controller; }}
namespace representation { class arena_cache; }
namespace support { class base_cache_manager; }
NS_START(support, depth1);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class depth1_metrics_aggregator
 * @ingroup support depth1
 *
 * @brief Aggregates and metrics collection for depth1 foraging. That
 * includes everything from \ref stateful_metrics_aggregator, and also:
 *
 * - FSM cache acquisition metrics
 * - Cache utilization metrics
 * - Cache lifecycle metrics
 * - World model metrics
 * - Task execution metrics (per task)
 * - TAB metrics (rooted at generalist)
 */
class depth1_metrics_aggregator : public depth0::stateful_metrics_aggregator,
                                  public er::client<depth1_metrics_aggregator> {
 public:
  using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

  depth1_metrics_aggregator(const struct params::metrics_params* params,
                            const std::string& output_root);

  /**
   * @brief Collect metrics from a finished or aborted task.
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
  void task_finish_or_abort_cb(const ta::polled_task* task);

  void task_alloc_cb(const ta::polled_task*, const ta::bi_tab* tab);

  /**
   * @brief Collect metrics from the depth1 controller.
   */
  template<class ControllerType>
  void collect_from_controller(const ControllerType* const controller) {
  auto worldm_m = dynamic_cast<const metrics::world_model_metrics*>(controller);
  auto manip_m =
      dynamic_cast<const metrics::blocks::manipulation_metrics*>(controller);
  auto movement_m =
      dynamic_cast<const metrics::fsm::movement_metrics*>(controller);

  ER_ASSERT(movement_m, "Controller does not provide FSM movement metrics");
  ER_ASSERT(worldm_m, "Controller does not provide world model metrics");
  ER_ASSERT(manip_m, "Controller does not provide block manipulation metrics");

  collect("fsm::movement", *movement_m);
  collect("blocks::manipulation", *manip_m);
  collect("perception::world_model", *worldm_m);

  if (nullptr != controller->current_task()) {
    auto collision_m = dynamic_cast<const metrics::fsm::collision_metrics*>(
        dynamic_cast<const ta::polled_task*>(controller->current_task())
        ->mechanism());
    auto block_acq_m =
        dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            dynamic_cast<const ta::polled_task*>(controller->current_task())
                ->mechanism());
    auto dist_m = dynamic_cast<const rcppsw::metrics::tasks::bi_tdgraph_metrics*>(
        controller);


    ER_ASSERT(block_acq_m,
              "Task does not provide FSM block acquisition metrics");
    ER_ASSERT(collision_m, "FSM does not provide collision metrics");
    ER_ASSERT(dist_m, "Controller does not provide task distribution metrics");

    collect("fsm::collision", *collision_m);
    collect_if(
        "blocks::acquisition",
        *dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            controller->current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kBlock ==
                 dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });
    collect_if(
        "caches::acquisition",
        *dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            controller->current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kExistingCache ==
                 dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });
    collect("tasks::distribution", *dist_m);
  }
}

  /**
   * @brief Collect utilization metrics from a cache in the arena.
   */
  void collect_from_cache(
      const representation::arena_cache* cache);

  /**
   * @brief Collect lifecycle metrics across all caches in the arena.
   */
  void collect_from_cache_manager(
      const support::base_cache_manager* manager);
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_DEPTH1_METRICS_AGGREGATOR_HPP_ */
