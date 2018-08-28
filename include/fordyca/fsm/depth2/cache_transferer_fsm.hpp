/**
 * @file cache_transferer_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_TRANSFERER_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_TRANSFERER_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/depth1/acquire_existing_cache_fsm.hpp"
#include "fordyca/fsm/block_transporter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class perceived_arena_map; }
namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm, depth2);
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_transferer_fsm
 * @ingroup fsm depth2
 *
 * @brief The FSM for a cache transferer task. Each robot executing this FSM
 * will acquire a cache (either a known cache or via random exploration), pickup
 * a block from it and then bring it to ANOTHER cache (either a known cache or
 * one found via random exploration) and drop it.
 */
class cache_transferer_fsm : public base_foraging_fsm,
                             public fsm::block_transporter,
                             public metrics::fsm::goal_acquisition_metrics,
                             public task_allocation::taskable,
                             public visitor::visitable_any<depth2::cache_transferer_fsm> {
 public:
  cache_transferer_fsm(
      std::shared_ptr<rcppsw::er::server> server,
      const controller::cache_selection_matrix* sel_matrix,
      controller::saa_subsystem* saa,
      representation::perceived_arena_map* map);

  /* taskable overrides */
  void task_reset(void) override { init(); }
  void task_start(const task_allocation::taskable_argument*) override {}
  void task_execute(void) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return m_task_running; }

  /* collision metrics */
  FSM_WRAPPER_DECLARE(bool, in_collision_avoidance);
  FSM_WRAPPER_DECLARE(bool, entered_collision_avoidance);
  FSM_WRAPPER_DECLARE(bool, exited_collision_avoidance);
  FSM_WRAPPER_DECLARE(uint, collision_avoidance_duration);

  /* goal acquisition metrics */
  FSM_WRAPPER_DECLARE(acquisition_goal_type, acquisition_goal);
  FSM_WRAPPER_DECLARE(bool, is_vectoring_to_goal);
  FSM_WRAPPER_DECLARE(bool, is_exploring_for_goal);
  FSM_WRAPPER_DECLARE(bool, goal_acquired);

  /* block transportation */
  FSM_WRAPPER_DECLARE(transport_goal_type, block_transport_goal);

  /**
   * @brief Reset the FSM.
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_SRC_CACHE,     /* superstate for finding source cache */
    ST_WAIT_FOR_BLOCK_PICKUP, /* wait for block pickup signal */
    ST_ACQUIRE_DEST_CACHE,    /* superstate for finding a dest cache */
    ST_WAIT_FOR_BLOCK_DROP,   /* wait for block drop signal go */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* depth2 foraging states */
  HFSM_STATE_DECLARE_ND(cache_transferer_fsm, start);
  HFSM_STATE_DECLARE_ND(cache_transferer_fsm, acquire_src_cache);
  HFSM_STATE_DECLARE(cache_transferer_fsm,
                     wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cache_transferer_fsm,
                        acquire_dest_cache);
  HFSM_STATE_DECLARE(cache_transferer_fsm,
                     wait_for_block_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cache_transferer_fsm, finished);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  // clang-format off
  bool                               m_task_running{false};
  depth1::acquire_existing_cache_fsm m_cache_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_TRANSFERER_FSM_HPP_ */
