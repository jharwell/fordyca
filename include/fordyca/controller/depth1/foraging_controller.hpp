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
#include "rcppsw/task_allocation/task_graph_vertex.hpp"
#include "fordyca/metrics/tasks/reactive_collator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class polled_executive;
class executable_task;
class task_decomposition_graph;
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
                            public virtual rcppsw::metrics::tasks::management_metrics,
                            public visitor::visitable_any<foraging_controller> {
 public:
  foraging_controller(void);

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  std::shared_ptr<tasks::base_foraging_task> current_task(void) const override;

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
  bool task_aborted(void) const override { return m_task_collator.task_aborted(); }
  bool has_new_allocation(void) const override { return m_task_collator.has_new_allocation(); }
  bool allocation_changed(void) const override { return m_task_collator.has_new_allocation(); }
  bool task_finished(void) const override { return m_task_collator.task_finished(); }
  double task_last_exec_time(void) const override { return m_task_collator.task_last_exec_time(); }

  std::string current_task_name(void) const override;
  bool employed_partitioning(void) const override;
  std::string subtask_selection(void) const override;

 protected:
  /* executive callbacks */
  void task_abort_cleanup(const ta::task_graph_vertex& task);
  void task_alloc_notify(const ta::task_graph_vertex& task);
  void task_finish_notify(const ta::task_graph_vertex& task);

 private:
  void tasking_init(params::depth0::stateful_foraging_repository* stateful_repo,
                    params::depth1::task_repository* task_repo);

  // clang-format off
  metrics::tasks::reactive_collator                  m_task_collator;
  bool                                               m_display_task{false};
  std::string                                        m_prev_task{""};
  std::unique_ptr<ta::polled_executive>              m_executive;
  std::shared_ptr<ta::task_decomposition_graph>      m_graph;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_CONTROLLER_HPP_ */
