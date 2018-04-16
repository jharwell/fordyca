/**
 * @file acquire_block_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <list>

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_for_block_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/metrics/fsm/stateful_metrics.hpp"
#include "fordyca/metrics/fsm/stateless_metrics.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct fsm_params;
}
namespace representation {
class perceived_arena_map;
class block;
} // namespace representation

namespace controller {
namespace depth0 {
class sensing_subsystem;
}
class actuation_subsystem;
} // namespace controller

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_block_fsm
 * @ingroup fsm
 *
 * @brief The FSM for an acquiring a free (i.e. not in a cache) block in the
 * arena.
 *
 * Each robot executing this FSM will look for a block (either a known block or
 * via stateless exploration). Once an existing block has been acquired, it
 * signals that it has completed its task.
 */
class acquire_block_fsm : public base_foraging_fsm,
                          public metrics::fsm::stateless_metrics,
                          public metrics::fsm::stateful_metrics,
                          public rcppsw::task_allocation::taskable {
 public:
  acquire_block_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      std::shared_ptr<representation::perceived_arena_map> map);

  acquire_block_fsm(const acquire_block_fsm& fsm) = delete;
  acquire_block_fsm& operator=(const acquire_block_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  bool task_finished(void) const override {
    return ST_FINISHED == current_state();
  }
  void task_reset(void) override { init(); }
  void task_start(
      const rcppsw::task_allocation::taskable_argument* const) override {}
  bool task_running(void) const override {
    return ST_ACQUIRE_BLOCK == current_state();
  }

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override { return false; }

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_BLOCK, /* superstate for finding a free block */
    ST_FINISHED,      /* A block has been acquired; wait to be reset */
    ST_MAX_STATES
  };

 private:
  /**
   * @brief Acquire a free block.
   *
   * @return TRUE if a block has been acquired, FALSE otherwise.
   */
  bool acquire_any_block(void);

  /**
   * @brief Acquire a known block. If the robot's knowledge of the chosen
   * block's existence expires during the pursuit of a known block, that is
   * ignored.
   *
   * @param blocks The list of perceived blocks. This CANNOT be a reference, as
   * the robot's list of perceived blocks can change during the course of
   * acquiring a block, and refering to specific positions within the vector
   * that the robot maintains leads to...interesting behavior.
   */
  bool acquire_known_block(std::list<representation::perceived_block> blocks);

  HFSM_STATE_DECLARE_ND(acquire_block_fsm, start);
  HFSM_STATE_DECLARE_ND(acquire_block_fsm, acquire_block);
  HFSM_STATE_DECLARE_ND(acquire_block_fsm, finished);

  HFSM_EXIT_DECLARE(acquire_block_fsm, exit_acquire_block);

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
  bool                                                 m_vectoring{false};
  const argos::CVector2                                mc_nest_center;
  std::shared_ptr<representation::block>               m_best_block{nullptr};
  argos::CRandom::CRNG*                                m_rng;
  std::shared_ptr<representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::er::server>                  m_server;
  vector_fsm                                           m_vector_fsm;
  explore_for_block_fsm                                m_explore_fsm;
  // clang-format on
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_ */
