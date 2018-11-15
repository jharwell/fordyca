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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GREEDY_PARTITIONING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GREEDY_PARTITIONING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth0/stateful_controller.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class bi_tdgraph_executive;
class bi_tab;
class executable_task;
class polled_task;
}}
namespace visitor = rcppsw::patterns::visitor;
namespace ta = rcppsw::task_allocation;

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
namespace depth1 { class controller_repository; }
}

NS_START(controller);
class cache_sel_matrix;
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
                                       public visitor::visitable_any<greedy_partitioning_controller>,
                                       public rcppsw::metrics::tasks::bi_tdgraph_metrics {
 public:
  greedy_partitioning_controller(void);
  ~greedy_partitioning_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  /* task distribution metrics */
  int current_task_depth(void) const override;
  int current_task_id(void) const override;
  int current_task_tab(void) const override;

  /* goal acquisition metrics */
  TASK_WRAPPER_DECLAREC(bool, goal_acquired);
  TASK_WRAPPER_DECLAREC(acquisition_goal_type, acquisition_goal);

  /* block transportation */
  TASK_WRAPPER_DECLAREC(transport_goal_type, block_transport_goal);

  /**
   * @brief Get the current task the controller is executing.
   */
  virtual tasks::base_foraging_task* current_task(void);
  virtual const tasks::base_foraging_task* current_task(void) const;

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

  const ta::bi_tab* active_tab(void) const;

  /*
   * Public to setup metric collection from tasks.
   */
  const ta::bi_tdgraph_executive* executive(void) const { return m_executive.get(); }
  ta::bi_tdgraph_executive* executive(void) { return m_executive.get(); }


 protected:
  const class cache_sel_matrix* cache_sel_matrix(void) const {
    return m_cache_sel_matrix.get();
  }
  /**
   * @brief Perform initialization that derived classes will also need to do.
   *
   * @param node XML node passed to \ref Init() function
   * @param param_repo Parameter repository for this controller.
   */
  void non_unique_init(ticpp::Element& node,
                       params::depth1::controller_repository* param_repo);

  /*
   * The \ref greedy_partitioning_controller owns the executive, but derived
   * classes can access it and set it to whatever they want. This is done to
   * reduce the amount of function overriding that would have to be performed
   * otherwise if derived controllers each had private executives--slightly
   * cleaner to do it this way I think.
   *
   * Strategy pattern!
   */
  void executive(std::unique_ptr<ta::bi_tdgraph_executive> executive);


 private:
  /**
   * @brief Callback for task abort. Task argument unused for now--only need to
   * know that a task WAS aborted. \see \ref task_aborted().
   */
  void task_abort_cb(const ta::polled_task*);

  // clang-format off
  bool                                      m_display_task{false};
  bool                                      m_task_aborted{false};
  std::unique_ptr<class cache_sel_matrix>   m_cache_sel_matrix;
  std::unique_ptr<ta::bi_tdgraph_executive> m_executive;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_GREEDY_PARTITIONING_CONTROLLER_HPP_ */
