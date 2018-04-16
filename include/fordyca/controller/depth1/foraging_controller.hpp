/**
 * @file depth1/foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "rcppsw/metrics/tasks/management_metrics.hpp"
#include "rcppsw/metrics/tasks/allocation_metrics.hpp"
#include "fordyca/controller/depth1/task_metrics_store.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class polled_executive;
class executable_task;
}}

NS_START(fordyca);
namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

namespace tasks {
class harvester;
class collector;
class generalist;
class foraging_task;
}

NS_START(controller, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class foraging_controller
 * @ingroup controller depth1
 *
 * @brief A foraging controller that switches between \ref generalist,
 * \ref harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 */
class foraging_controller : public depth0::stateful_foraging_controller,
                            public rcppsw::metrics::tasks::management_metrics,
                            public visitor::visitable_any<foraging_controller> {
 public:
  foraging_controller(void);

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  tasks::foraging_task* current_task(void) const;
  bool is_transporting_to_nest(void) const override;

  /**
   * @brief If \c TRUE, then a robot has acquired a cache, meaning that it has
   * arrived to one via some mechanism.
   *
   * This state corresponds to one of the FSMs within the controller waiting for
   * a signal from the simulation that in order to move to the next stage of its
   * task.
   */
  bool cache_acquired(void) const;

  /**
   * @brief If \c TRUE, then a robot has acquired a block, meaning that it has
   * arrived to one via some mechanism.
   *
   * This state corresponds to one of the FSMs within the controller waiting for
   * a signal from the simulation that in order to move to the next stage of its
   * task.
   */
  bool block_acquired(void) const;

  /**
   * @brief Set whether or not a robot is supposed to display the task it is
   * currently working on above itself during simulation.
   */
  void display_task(bool display_task) { m_display_task = display_task; }

  /**
   * @brief If \c TRUE, then the robot should display the task it is currently
   * working on above itself during simulation.
   */
  bool display_task(void) const { return m_display_task; }

  /* task metrics */
  bool has_aborted_task(void) const override { return m_metric_store.task_aborted; }
  bool has_new_allocation(void) const override { return m_metric_store.task_alloc; }
  bool has_changed_allocation(void) const override { return m_metric_store.alloc_sw; }
  bool has_finished_task(void) const override { return m_metric_store.task_finish; }
  double last_task_exec_time(void) const override { return m_metric_store.last_task_exec_time; }
  std::string current_task_name(void) const override;
  bool employed_partitioning(void) const override;
  std::string subtask_selection(void) const override;

 private:
  void task_abort_cleanup(task_allocation::executable_task*);
  void task_alloc_notify(task_allocation::executable_task*);
  void task_finish_notify(task_allocation::executable_task*);

  // clang-format off
  struct task_metrics_store                          m_metric_store;
  bool                                               m_display_task{false};
  std::string                                        m_prev_task{""};
  std::unique_ptr<task_allocation::polled_executive> m_executive;
  std::unique_ptr<tasks::harvester>                  m_harvester;
  std::unique_ptr<tasks::collector>                  m_collector;
  std::unique_ptr<tasks::generalist>                 m_generalist;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_CONTROLLER_HPP_ */
