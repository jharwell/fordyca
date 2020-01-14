/**
 * \file bitd_dpo_controller.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_DPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_DPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "fordyca/tasks/task_status.hpp"

#include "cosm/ta/logical_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta {
class bi_tdgraph_executive;
class executable_task;
class polled_task;
namespace ds {
class bi_tab;
} /* namespace ds */
}

NS_START(fordyca);

namespace config {
namespace depth1 { class controller_repository; }
}

NS_START(controller);
class cache_sel_matrix;
class dpo_perception_subsystem;
NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bitd_dpo_controller
 * \ingroup controller depth1
 *
 * \brief A controller defining the task allocation space via BIfurcating Task
 * Decomposition (BITD) and spliting the \ref generalist task into the \ref
 * harvester, and \ref collector tasks, according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 *
 * Uses a DPO data store for tracking arena state and object relavance.
 */
class bitd_dpo_controller : public depth0::dpo_controller,
                            public rer::client<bitd_dpo_controller>,
                            public ctametrics::bi_tdgraph_metrics {
 public:
  using dpo_controller::perception;

  bitd_dpo_controller(void) RCSW_COLD;
  ~bitd_dpo_controller(void) override RCSW_COLD;

  bitd_dpo_controller(const bitd_dpo_controller&) = delete;
  bitd_dpo_controller& operator=(const bitd_dpo_controller&) = delete;

  /* base_controller overrides */
  void init(ticpp::Element& node) override RCSW_COLD;
  void control_step(void) override;
  std::type_index type_index(void) const override { return typeid(*this); }

  /* task distribution metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(int, current_task_depth, const);
  RCPPSW_WRAP_OVERRIDE_DECL(int, current_task_id, const final);
  RCPPSW_WRAP_OVERRIDE_DECL(int, current_task_tab, const);

  /* goal acquisition metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const final);
  RCPPSW_WRAP_OVERRIDE_DECL(cfmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const final);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(fsm::foraging_transport_goal::type,
                            block_transport_goal,
                            const final);

  /**
   * \brief Get the current task the controller is executing.
   */
  tasks::base_foraging_task* current_task(void) RCSW_PURE {
    return m_current_task;
  }
  const tasks::base_foraging_task* current_task(void) const RCSW_PURE {
    return m_current_task;
  }

  int task_id(const std::string& task_name) const;

  /**
   * \brief Set whether or not a robot is supposed to display the task it is
   * currently working on above itself during simulation.
   */
  void display_task(bool display_task) { m_display_task = display_task; }

  /**
   * \brief If \c TRUE, then the robot should display the task it is currently
   * working on above itself during simulation.
   */
  bool display_task(void) const { return m_display_task; }

  const cta::ds::bi_tab* active_tab(void) const RCSW_PURE;

  /*
   * Public to setup metric collection from tasks.
   */
  const cta::bi_tdgraph_executive* executive(void) const { return m_executive.get(); }
  cta::bi_tdgraph_executive* executive(void) { return m_executive.get(); }

  /**
   * \brief Get whether or not a task has been aborted this timestep.
   *
   * This functionality CANNOT use the abort state of the \ref current_task()
   * because as soon as a task is aborted, the executive allocates a new task
   * the *same* timestep, and so when the loop functions check if a task has
   * been aborted, using the current task's abort status will always return
   * false, and lead to inconsistent simulation state.
   */
  tasks::task_status task_status(void) const { return m_task_status; }

  const class cache_sel_matrix* cache_sel_matrix(void) const {
    return m_cache_sel_matrix.get();
  }
  class cache_sel_matrix* cache_sel_matrix(void) {
    return m_cache_sel_matrix.get();
  }

  void task_status_update(tasks::task_status s) { m_task_status = s; }

 protected:
  /**
   * \brief Initialization that derived classes may also need to perform, if
   * they want to use any of the following parts of this class's functionality
   * as-is:
   *
   * - Block selection matrix (\ref block_sel_matrix)
   * - Cache selection matrix (\ref cache_sel_matrix)
   * - Task executive (\ref cta::bi_tdgraph_executive)
   * - DPO perception subsystem (\ref dpo_perception_subsystem)
   *
   * \param config_repo Handle to parameter repository for this controller
   *                   (after parsing and validation).
   */
  void shared_init(const config::depth1::controller_repository& config_repo) RCSW_COLD;

  /*
   * The \ref bitd_dpo_controller owns the executive, but derived classes can
   * access it and set it to whatever they want (strategy pattern). This is done
   * to reduce the amount of function overriding that would have to be performed
   * otherwise if derived controllers each had private executives.
   */
  void executive(std::unique_ptr<cta::bi_tdgraph_executive> executive);

  /**
   * \brief Callback for task abort. Task argument unused for now--only need to
   * know that a task WAS aborted. \see \ref task_aborted().
   */
  void task_abort_cb(const cta::polled_task*);

  /**
   * \brief Callback for task start. Needed to reset the task state of the
   * controller (not the task, which is handled by the executive) in the case
   * that the previous task was aborted. Not reseting this results in erroneous
   * handling of the newly allocated task as if it was aborted by the loop
   * functions, resulting in inconsistent state with the robot's executive.
   */
  void task_start_cb(cta::polled_task* task);

 private:
  void private_init(const config::depth1::controller_repository& config_repo) RCSW_COLD;

  /* clang-format off */
  bool                                       m_display_task{false};

  /**
   * \brief The current task the controller is executing. This is also tracked
   * by the executive, so it might seem redundant to also track it here. This is
   * done to avoid having to dynamically cast from \ref cta::polled_task to \ref
   * tasks::base_foraging_task on multiple times EVERY timestep for EVERY robot,
   * which was enough to make it show up in VTune as a minor bottleneck. See
   * #547.
   */
  tasks::base_foraging_task*                 m_current_task{nullptr};
  tasks::task_status                         m_task_status{tasks::task_status::ekNULL};
  std::unique_ptr<class cache_sel_matrix>    m_cache_sel_matrix;
  std::unique_ptr<cta::bi_tdgraph_executive> m_executive;
  /* clang-format on */
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_BITD_DPO_CONTROLLER_HPP_ */
