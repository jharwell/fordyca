/**
 * @file cache_finisher_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_FINISHER_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_FINISHER_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/metrics/fsm/block_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/cache_acquisition_metrics.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/depth2/acquire_new_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }
namespace representation { class perceived_arena_map; }
namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_finisher_fsm
 * @ingroup fsm depth2
 *
 * @brief The FSM for a cache finisher task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and then bring it to ANOTHER block (either a known block or
 * one found via random exploration) and drop it to create a new cache.
 */
class cache_finisher_fsm : public base_foraging_fsm,
                           public metrics::fsm::block_acquisition_metrics,
                           public metrics::fsm::cache_acquisition_metrics,
                           public task_allocation::taskable,
                           public visitor::visitable_any<depth2::cache_finisher_fsm> {
 public:
  cache_finisher_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      const std::shared_ptr<representation::perceived_arena_map>& map);

  /* taskable overrides */
  void task_reset(void) override { init(); }
  void task_start(const task_allocation::taskable_argument*) override {}
  void task_execute(void) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return m_task_running; }

  /* base FSM metrics */
  bool is_avoiding_collision(void) const override;

  /* block acquisition metrics */
  bool is_exploring_for_block(void) const override;
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;
  bool block_acquired(void) const override;

  /* cache acquisition metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool cache_acquired(void) const override;

  /**
   * @brief Reset the FSM.
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_BLOCK,         /* superstate for finding a block */
    ST_WAIT_FOR_BLOCK_PICKUP, /* wait for block pickup signal */
    ST_ACQUIRE_NEW_CACHE,    /* superstate for finding a new cache */
    ST_WAIT_FOR_BLOCK_DROP,  /* wait for block drop signal go */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /**
   * @brief It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block).
   *
   * In that case, this timeout will cause the robot to try again to acquire a
   * block, and because of the decaying relevance of cells, it will eventually
   * pick a different block than the one that got it into this predicament, and
   * the system will be able to continue profitably.
   */
  constexpr static uint kPICKUP_TIMEOUT = 100;

  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* depth2 foraging states */
  HFSM_STATE_DECLARE_ND(cache_finisher_fsm, start);
  HFSM_STATE_DECLARE_ND(cache_finisher_fsm, acquire_block);
  HFSM_STATE_DECLARE(cache_finisher_fsm,
                     wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cache_finisher_fsm,
                        acquire_new_cache);
  HFSM_STATE_DECLARE(cache_finisher_fsm,
                     wait_for_block_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cache_finisher_fsm, finished);

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
  bool                  m_task_running{false};
  uint                  m_pickup_count{0};
  acquire_block_fsm     m_block_fsm;
  acquire_new_cache_fsm m_cache_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_CACHE_FINISHER_FSM_HPP_ */
