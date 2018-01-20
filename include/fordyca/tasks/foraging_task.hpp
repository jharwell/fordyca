/**
 * @file foraging_task.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_FORAGING_TASK_HPP_
#define INCLUDE_FORDYCA_TASKS_FORAGING_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/metrics/collectible_metrics/fsm/depth1_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/stateful_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/stateless_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/task_metrics.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace events {
class cached_block_pickup;
class cache_block_drop;
class free_block_pickup;
class nest_block_drop;
class cache_vanished;
}

namespace visitor = rcppsw::patterns::visitor;

NS_START(tasks);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class foraging_task
 * @ingroup tasks
 *
 * @brief Interface specifying the visit set for all foraging tasks in FORDYCA.
 */
class foraging_task
    : public metrics::collectible_metrics::fsm::stateless_metrics,
      public metrics::collectible_metrics::fsm::stateful_metrics,
      public metrics::collectible_metrics::fsm::depth1_metrics,
      public metrics::collectible_metrics::task_metrics,
      public visitor::polymorphic_visitable<foraging_task,
                                            events::cached_block_pickup,
                                            events::cache_block_drop,
                                            events::free_block_pickup,
                                            events::nest_block_drop,
                                            events::cache_vanished> {
 public:
  static constexpr char kCollectorName[] = "Collector";
  static constexpr char kForagerName[] = "Forager";
  static constexpr char kGeneralistName[] = "Generalist";

  explicit foraging_task(const std::string& name) : mc_name(name) {}

  /**
   * @brief If \c TRUE, then a robot has acquired a cache and is waiting for the
   * block pickup/block drop signal from the arena.
   */
  virtual bool cache_acquired(void) const = 0;

  /**
   * @brief If \c TRUE, then a robot has acquired a block and is waiting for the
   * block pickup signal from the arena.
   */
  virtual bool block_acquired(void) const = 0;

  std::string task_name(void) const override { return mc_name; }

 private:
  const std::string mc_name;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_FORAGING_TASK_HPP_ */
