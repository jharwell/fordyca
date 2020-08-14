/**
 * \file cache_finisher.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "fordyca/events/free_block_interactor.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_finisher
 * \ingroup tasks depth2
 *
 * \brief Task in which robots locate a free block and drop it next to/on top of
 * a free block in the arena to finish the creation of a new cache. It is
 * abortable, and has one task interface.
 */
class cache_finisher final : public foraging_task,
                       public events::free_block_interactor,
                       public events::dynamic_cache_interactor,
                       public rer::client<cache_finisher> {
 public:
  cache_finisher(const struct cta::config::task_alloc_config* config,
                 std::unique_ptr<cta::taskable> mechanism);

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a \ref free_block_interactor, you have no way to way
   * which depth2 task the object ACTUALLY is without using a set of if()
   * statements, which is a brittle design. This is not the cleanest, but is
   * still more elegant than the alternative.
   */
  void accept(events::detail::robot_free_block_drop& visitor) override;
  void accept(events::detail::robot_free_block_pickup& visitor) override;
  void accept(events::detail::block_vanished& visitor) override;
  void accept(events::detail::block_proximity&) override {};
  void accept(events::detail::cache_proximity& visitor) override;

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
  bool task_completed(void) const override { return task_finished(); }

  void task_start(cta::taskable_argument*) override;
  double abort_prob_calc(void) override RCSW_PURE;
  rtypes::timestep interface_time_calc(size_t interface,
                                       const rtypes::timestep& start_time) override RCSW_PURE;
  void active_interface_update(int) override;
};

NS_END(depth2, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_ */
