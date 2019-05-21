/**
 * @file harvester.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/events/free_block_interactor.hpp"

#include "rcppsw/ta/abort_probability.hpp"
#include "rcppsw/ta/polled_task.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class harvester
 * @ingroup fordyca tasks depth1
 *
 * @brief Task in which robots locate a free block and bring it to a known
 * cache. It is abortable, and has one task interface.
 */
class harvester final : public foraging_task,
                  public events::existing_cache_interactor,
                  public events::free_block_interactor,
                  public rer::client<harvester> {
 public:
  harvester(const struct rta::config::task_alloc_config* config,
            std::unique_ptr<rta::taskable> mechanism);
  ~harvester(void) override = default;

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a \ref existing_cache_interactor, you have no way to
   * way which task the object ACTUALLY is without using a set of if()
   * statements, which is a brittle design. This is not the cleanest, but is
   * still more elegant than the alternative.
   */
  void accept(events::detail::free_block_pickup& visitor) override;
  void accept(events::detail::free_block_drop&) override {}
  void accept(events::detail::block_vanished&) override;

  void accept(events::detail::cache_block_drop& visitor) override;
  void accept(events::detail::cached_block_pickup&) override {}
  void accept(events::detail::cache_vanished& visitor) override;

  /* goal acquisition metrics */
  TASK_WRAPPER_DECLARE(bool, goal_acquired, const override);
  TASK_WRAPPER_DECLARE(exp_status, is_exploring_for_goal, const override);
  TASK_WRAPPER_DECLARE(bool, is_vectoring_to_goal, const override);
  TASK_WRAPPER_DECLARE(acq_goal_type, acquisition_goal, const override);
  TASK_WRAPPER_DECLARE(rmath::vector2u, acquisition_loc, const override);
  TASK_WRAPPER_DECLARE(rmath::vector2u, current_explore_loc, const override);
  TASK_WRAPPER_DECLARE(rmath::vector2u, current_vector_loc, const override);

  /* block transportation */
  TASK_WRAPPER_DECLARE(transport_goal_type, block_transport_goal, const override);

  /* task metrics */
  bool task_at_interface(void) const override;
  bool task_completed(void) const override { return task_finished(); }

  void task_start(const rta::taskable_argument*) override;
  double abort_prob_calc(void) override;
  double interface_time_calc(uint interface,
                             double start_time) override;
  void active_interface_update(int) override;
};

NS_END(depth1, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_ */
