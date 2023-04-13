/**
 * \file generalist.hpp
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
 * \ingroup tasks d0
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
class generalist final : public rer::client<generalist>,
                         public foraging_task {
 public:
  generalist(const cta::config::task_alloc_config* config,
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
  void accept(fccd0::events::free_block_pickup& visitor) override;
  void accept(fccd0::events::free_block_drop& ) override {}
  void accept(fccd0::events::block_vanished& visitor) override;

  void accept(fccd1::events::free_block_pickup& visitor) override;
  void accept(fccd1::events::free_block_drop& ) override {}
  void accept(fccd1::events::block_vanished& visitor) override;

  void accept(fccd2::events::free_block_pickup& visitor) override;
  void accept(fccd2::events::free_block_drop& ) override {}
  void accept(fccd2::events::block_vanished& visitor) override;

  /* nest interaction events */
  void accept(fccd0::events::nest_block_drop& visitor) override;
  void accept(fccd1::events::nest_block_drop& visitor) override;
  void accept(fccd2::events::nest_block_drop& visitor) override;

  /* goal acquisition metrics */
  RCPPSW_WRAP_DECL_OVERRIDE(bool, goal_acquired, const);
  RCPPSW_WRAP_DECL_OVERRIDE(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, acquisition_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(boost::optional<rmath::vector3z>, vector_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_DECL_OVERRIDE(fsm::foraging_transport_goal,
                            block_transport_goal,
                            const);
  bool is_phototaxiing_to_goal(bool include_ca) const override RCPPSW_PURE;

  /* block carrying */
  RCPPSW_WRAP_DECL_OVERRIDE(const cssblocks::drop::base_drop*,
                            block_drop_strategy,
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
