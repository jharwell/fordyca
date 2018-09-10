/**
 * @file metrics_aggregator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_METRICS_AGGREGATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_METRICS_AGGREGATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/stateful_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class polled_task;
class bifurcating_tab;
}}
namespace ta = rcppsw::task_allocation;
NS_START(fordyca);

namespace controller { namespace depth1 { class foraging_controller; }}
namespace representation { class arena_cache; }
namespace metrics { namespace caches { class lifecycle_collator; }}

NS_START(support, depth1);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class metrics_aggregator
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
class metrics_aggregator : public depth0::stateful_metrics_aggregator,
                           public er::client<metrics_aggregator> {
 public:
  metrics_aggregator(const struct params::metrics_params* params,
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

  void task_alloc_cb(const ta::polled_task*, const ta::bifurcating_tab* tab);

  /**
   * @brief Collect metrics from the depth1 controller.
   */
  void collect_from_controller(
      const controller::depth1::foraging_controller* controller);

  /**
   * @brief Collect utilization metrics from a cache in the arena.
   */
  void collect_from_cache(
      const representation::arena_cache* cache);

  /**
   * @brief Collect lifecycle metrics across all caches in the arena.
   */
  void collect_from_cache_collator(
      const metrics::caches::lifecycle_collator* collator);
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_METRICS_AGGREGATOR_HPP_ */
