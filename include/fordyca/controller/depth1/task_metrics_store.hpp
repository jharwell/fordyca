/**
 * @file task_metrics_store.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_METRICS_STORE_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_METRICS_STORE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/

/**
 * @struct task_metrics_store
 * @ingroup controller depth1
 *
 * @brief A collection of variables used in reporting metrics about tasks at the
 * depth 1 level internally in the controller.
 */
struct task_metrics_store {
  void reset(void) {
    task_aborted = false;
    task_alloc = false;
    alloc_sw = false;
    task_finish = false;
    last_task_exec_time = 0.0;
  }

  bool task_aborted{false};  /// Was the current task aborted?
  bool task_alloc{false};    /// Was a task allocated on the current timestep?

  /**
   * Did a robot's task allocation change this timestep?
   */
  bool alloc_sw{false};
  bool task_finish{false};   /// Was a task finished on this timestep?

  /**
   * How long did the most recently finished task take?
   */
  double last_task_exec_time{0.0};
};


NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_METRICS_STORE_HPP_ */
