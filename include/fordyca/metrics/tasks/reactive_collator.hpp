/**
 * @file reactive_collator.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TASKS_REACTIVE_COLLATOR_HPP_
#define INCLUDE_FORDYCA_METRICS_TASKS_REACTIVE_COLLATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tasks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class reactive_collator
 * @ingroup metrics tasks
 *
 * @brief Collates information about the currently running task for the purpose
 * of metric collection.
 */
class reactive_collator {
 public:
  reactive_collator(void) = default;

  bool task_aborted(void) const { return m_task_aborted; }
  void task_aborted(bool b) { m_task_aborted = b; }
  bool task_finished(void) const { return m_task_finish; }
  void task_finished(bool b) { m_task_finish = b; }
  bool has_new_allocation(void) const { return m_task_alloc; }
  void has_new_allocation(bool b) { m_task_alloc = b; }
  bool allocation_changed(void) const { return m_alloc_sw; }
  void allocation_changed(bool b) { m_alloc_sw = b; }
  double last_task_exec_time(void) const { return m_last_task_exec_time; }
  void last_task_exec_time(double d) { m_last_task_exec_time = d; }

  void reset(void) {
    m_task_aborted = false;
    m_task_alloc = false;
    m_alloc_sw = false;
    m_task_finish = false;
    m_last_task_exec_time = 0.0;
  }

 private:
  bool m_task_aborted{false};  /// Was the current task aborted?
  bool m_task_alloc{false};    /// Was a task allocated on the current timestep?

  /**
   * Did a robot's task allocation change this timestep?
   */
  bool m_alloc_sw{false};
  bool m_task_finish{false};   /// Was a task finished on this timestep?

  /**
   * How long did the most recently finished task take?
   */
  double m_last_task_exec_time{0.0};
};

NS_END(tasks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TASKS_REACTIVE_COLLATOR_HPP_ */
