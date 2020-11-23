/**
 * \file generalist.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_D0_GENERALIST_HPP_
#define INCLUDE_FORDYCA_TASKS_D0_GENERALIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/tasks/d0/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d0);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class generalist
 * \ingroup tasks
 *
 * \brief Class representing depth 0 task allocation: Perform the whole foraging
 * task: (1) Find a free block, and (2) bring it to the nest.
 *
 * It is decomposable into two subtasks that result in the same net change to
 * the arena state when run in sequence (possibly by two different robots):
 * \ref collector and \ref harvester. It is not abortable at task interfaces,
 * because it does not have any, but it IS still abortable if its current
 * execution time takes too long (as configured by parameters).
 */
class generalist final : public foraging_task {
 public:
  generalist(const cta::config::task_alloc_config* config,
             std::unique_ptr<cta::taskable> mechanism);

  /* event handling */
  void accept(events::detail::robot_free_block_pickup& visitor) override;
  void accept(events::detail::robot_free_block_drop&) override {}
  void accept(events::detail::robot_nest_block_drop& visitor) override;
  void accept(events::detail::block_vanished& visitor) override;

  /* goal acquisition metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const);
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);

  /* task metrics */
  bool task_at_interface(void) const override { return false; }
  bool task_completed(void) const override { return task_finished(); }

  void task_start(cta::taskable_argument* const) override {}

  rtypes::timestep current_time(void) const override RCPPSW_PURE;
  rtypes::timestep interface_time_calc(size_t,
                                       const rtypes::timestep&) override {
    return rtypes::timestep(0);
  }
  void active_interface_update(int) override {}
  double abort_prob_calc(void) override RCPPSW_PURE;
};

NS_END(d0, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_D0_GENERALIST_HPP_ */
