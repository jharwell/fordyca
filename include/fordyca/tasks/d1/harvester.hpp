/**
 * \file harvester.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"

#include "cosm/ta/abort_probability.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/tasks/d1/foraging_task.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/events/free_block_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class harvester
 * \ingroup tasks d1
 *
 * \brief Task in which robots locate a free block and bring it to a known
 * cache. It is abortable, and has one task interface.
 */
class harvester final : public foraging_task,
                        public events::existing_cache_interactor,
                        public events::free_block_interactor,
                        public rer::client<harvester> {
 public:
  harvester(const struct cta::config::task_alloc_config* config,
            std::unique_ptr<cta::taskable> mechanism);
  ~harvester(void) override = default;

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a given task which is a member of a set of tasks
   * which all implement the same interface, you have no way to way which task
   * the object ACTUALLY is without using a set of if() statements, which is a
   * brittle design. This is not the cleanest, but is still more elegant than
   * the alternative.
   *
   * I wish you didn't have to stub out the d0 and d2 events.
   */

  /* free block interaction events */
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::free_block_drop);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::free_block_pickup);
  RCPPSW_VISITOR_ACCEPT_STUB(fccd0::events::block_vanished);

  void accept(fccd1::events::free_block_pickup& visitor) override;
  void accept(fccd1::events::free_block_drop&) override {}
  void accept(fccd1::events::block_vanished& visitor) override;

  void accept(fccd2::events::free_block_pickup& visitor) override;
  void accept(fccd2::events::free_block_drop&) override {}
  void accept(fccd2::events::block_vanished& visitor) override;

  /* cache interaction events */
  void accept(fccd1::events::cache_block_drop& visitor) override;
  RCPPSW_VISITOR_ACCEPT_STUB(fccd1::events::cached_block_pickup);
  void accept(fccd1::events::cache_vanished& visitor) override;

  void accept(fccd2::events::cache_block_drop& visitor) override;
  RCPPSW_VISITOR_ACCEPT_STUB(fccd2::events::cached_block_pickup);
  void accept(fccd2::events::cache_vanished& visitor) override;

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

  /* block carrying */
  RCPPSW_WRAP_DECL_OVERRIDE(const cssblocks::drop::base_drop*,
                            block_drop_strategy,
                            const);

  /* task metrics */
  bool task_at_interface(void) const override RCPPSW_PURE;
  bool task_completed(void) const override { return task_finished(); }

  void task_start(cta::taskable_argument*) override;
  double abort_prob_calc(void) override RCPPSW_PURE;
  rtypes::timestep interface_time_calc(size_t interface,
                                       const rtypes::timestep& start_time) override RCPPSW_PURE;
  void active_interface_update(int) override;
};

NS_END(d1, tasks, fordyca);
