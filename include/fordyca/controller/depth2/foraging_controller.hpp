/**
 * @file depth2/foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_FORAGING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_FORAGING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth1/foraging_controller.hpp"
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

namespace tasks { namespace depth2 {
class foraging_task;
}}

NS_START(controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class foraging_controller
 * @ingroup controller depth2
 *
 * @brief A foraging controller that moves through a depth2 recursive task
 * decomposition graph, changing task according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 */
class foraging_controller : public depth1::foraging_controller,
                            public visitor::visitable_any<foraging_controller> {
 public:
  foraging_controller(void);

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;


  tasks::base_foraging_task* current_task(void) const override {
    return nullptr; /* TODO: Fixme! */
  } /* current_task() */

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
  struct depth1::task_metrics_store                  m_metric_store;
  bool                                               m_display_task{false};
  std::string                                        m_prev_task{""};
  std::unique_ptr<task_allocation::polled_executive> m_executive;
  // clang-format on
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_CONTROLLER_HPP_ */
