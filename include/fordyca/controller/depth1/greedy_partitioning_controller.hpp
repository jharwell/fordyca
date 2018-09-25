/**
 * @file greedy_partitioning_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_GREEDY_PARTITIONING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_GREEDY_PARTITIONING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth0/stateful_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class bifurcating_tdgraph_executive;
class polled_task;
}}

NS_START(fordyca);
namespace visitor = rcppsw::patterns::visitor;
namespace ta = rcppsw::task_allocation;

namespace tasks {
namespace depth0 { class generalist; }
namespace depth1 {
class harvester;
class collector;
class foraging_task;
}
}
namespace params {
namespace depth0 { class stateful_foraging_repository; }
namespace depth1 { class task_repository; }
}

NS_START(controller);
class cache_selection_matrix;
NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class greedy_partitioning_controller
 * @ingroup controller depth1
 *
 * @brief A greedy_partitioning controller that switches between \ref generalist,
 * \ref harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 */
class greedy_partitioning_controller : public depth0::stateful_controller,
                                       public er::client<greedy_partitioning_controller>,
                                       public visitor::visitable_any<greedy_partitioning_controller> {
 public:
  greedy_partitioning_controller(void);
  ~greedy_partitioning_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  tasks::base_foraging_task* current_task(void) override;
  const tasks::base_foraging_task* current_task(void) const override;

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

  /**
   * @brief Get whether or not a task has been aborted this timestep.
   *
   * This functionality CANNOT use the abort state of the \ref current_task()
   * because as soon as a task is aborted, the executive allocates a new task
   * the *same* timestep, and so when the loop functions check if a task has
   * been aborted, using the current task's abort status will always return
   * false, and lead to inconsistent simulation state.
   */
  bool task_aborted(void) const { return m_task_aborted; }
  void task_aborted(bool task_aborted) { m_task_aborted = task_aborted; }

  /**
   * @brief Callback for task abort. Task argument unused for now--only need to
   * know that a task WAS aborted. \see \ref task_aborted().
   */
  void task_abort_cb(const ta::polled_task*);

 protected:
  const cache_selection_matrix*  cache_sel_matrix(void) const {
    return m_cache_sel_matrix.get();
  }

 private:
  void tasking_init(params::depth0::stateful_foraging_repository* stateful_repo,
                    params::depth1::task_repository* task_repo);

  // clang-format off
  bool                                    m_display_task{false};
  bool                                    m_task_aborted{false};
  std::unique_ptr<cache_selection_matrix> m_cache_sel_matrix;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_GREEDY_PARTITIONING_CONTROLLER_HPP_ */
