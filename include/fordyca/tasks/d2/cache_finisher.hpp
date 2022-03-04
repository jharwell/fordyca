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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/tasks/d2/foraging_task.hpp"
#include "fordyca/events/free_block_interactor.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_finisher
 * \ingroup tasks d2
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
   * because when visiting a given task which is a member of a set of tasks
   * which all implement the same interface, you have no way to way which task
   * the object ACTUALLY is without using a set of if() statements, which is a
   * brittle design. This is not the cleanest, but is still more elegant than
   * the alternative.
   *
   * I wish you didn't have to stub out the d1 and d2 events.
   */
  /* free block interaction events */
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::free_block_drop);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::free_block_pickup);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::block_vanished);

  RCPPSW_VISITOR_ACCEPT_STUB(fccd1::events::free_block_drop);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd1::events::free_block_pickup);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd1::events::block_vanished);

  void accept(fccd2::events::free_block_drop& visitor) override;
  void accept(fccd2::events::free_block_pickup& visitor) override;
  void accept(fccd2::events::block_vanished& visitor) override;

  /* dynamic cache interaction events */
  void accept(fccd2::events::block_proximity&) override {}
  void accept(fccd2::events::cache_proximity&) override;

  /* goal acquisition metrics */
  RCPPSW_WRAP_DECL_OVERRIDE(bool, goal_acquired, const);
  RCPPSW_WRAP_DECL_OVERRIDE(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_DECL_OVERRIDE(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);
  bool is_phototaxiing_to_goal(bool) const override { return false; }

  /* task metrics */
  bool task_completed(void) const override { return task_finished(); }

  void task_start(cta::taskable_argument*) override;
  double abort_prob_calc(void) override RCPPSW_PURE;
  rtypes::timestep interface_time_calc(size_t interface,
                                       const rtypes::timestep& start_time) override RCPPSW_PURE;
  void active_interface_update(int) override;
};

NS_END(d2, tasks, fordyca);

