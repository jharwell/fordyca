/**
 * @file cache_transferer.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_TRANSFERER_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_TRANSFERER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_transferer
 * @ingroup fordyca tasks depth2
 *
 * @brief Task in which robots locate an existing cache, pickup a block from it,
 * and then transfer the block to a cache with higher utility (presumably one
 * that is closer to the nest). It is abortable, and has two task interfaces:
 * one at each cache it interacts with.
 */
class cache_transferer final : public foraging_task,
                         public events::existing_cache_interactor,
                         rer::client<cache_transferer> {
 public:
  cache_transferer(const struct rta::config::task_alloc_config* config,
                   std::unique_ptr<rta::taskable> mechanism);

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a \ref existing_cache_interactor, you have no way to
   * way which depth2 task the object ACTUALLY is without using a set of if()
   * statements, which is a brittle design. This is not the cleanest, but is
   * still more elegant than the alternative.
   */
  void accept(events::detail::cache_block_drop& visitor) override;
  void accept(events::detail::cached_block_pickup& visitor) override;
  void accept(events::detail::cache_vanished& visitor) override;

  /* goal acquisition metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const);
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(acq_goal_type, acquisition_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, acquisition_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_explore_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_vector_loc, const);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(transport_goal_type, block_transport_goal, const);

  /* task metrics */
  bool task_completed(void) const override { return task_finished(); }

  void task_start(const rta::taskable_argument*) override;
  double abort_prob_calc(void) override RCSW_PURE;
  double interface_time_calc(uint interface,
                             double start_time) override RCSW_PURE;
  void active_interface_update(int) override;
};

NS_END(depth2, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_TRANSFERER_HPP_ */
