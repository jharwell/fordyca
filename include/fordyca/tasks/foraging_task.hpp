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
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "fordyca/tasks/argument.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/random_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth0_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth1_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/task_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace events {
class cached_block_pickup;
class cache_block_drop;
class free_block_pickup;
class nest_block_drop;
}

namespace visitor = rcppsw::patterns::visitor;

NS_START(tasks);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
class foraging_task : public metrics::collectible_metrics::robot_metrics::random_metrics,
                      public metrics::collectible_metrics::robot_metrics::depth0_metrics,
                      public metrics::collectible_metrics::robot_metrics::depth1_metrics,
                      public metrics::collectible_metrics::task_metrics,
                      public visitor::polymorphic_visitable<foraging_task,
                                                            events::cached_block_pickup,
                                                            events::cache_block_drop,
                                                            events::free_block_pickup,
                                                            events::nest_block_drop> {
 public:
  foraging_task(void) {}

  virtual bool cache_acquired(void) const = 0;
  virtual bool block_acquired(void) const = 0;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_FORAGING_TASK_HPP_ */
