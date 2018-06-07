/**
 * @file acquire_goal_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_GOAL_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_GOAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/explore_for_goal_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class perceived_arena_map; }

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_goal_fsm
 * @ingroup fsm
 *
 * @brief The base FSM for an acquiring a goal in the arena.
 *
 * Each robot executing this FSM will acquire an instance of its goal (either a
 * known instance or via random exploration). Once the instance has been
 * acquired, it signals that it has completed its task.
 */
class acquire_goal_fsm : public base_foraging_fsm,
                         public metrics::fsm::goal_acquisition_metrics,
                         public rcppsw::task_allocation::taskable {
 public:
  acquire_goal_fsm(
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      std::shared_ptr<const representation::perceived_arena_map> map,
      std::function<bool(void)> goal_detect);

  acquire_goal_fsm(const acquire_goal_fsm& fsm) = delete;
  acquire_goal_fsm& operator=(const acquire_goal_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(__unused const rcppsw::task_allocation::taskable_argument* const arg) override {}
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return ST_ACQUIRE_GOAL == current_state(); }
  void task_reset(void) override { init(); }

  /* base FSM metrics */
  bool is_avoiding_collision(void) const override;

  /* goal acquisition metrics */
  bool is_exploring_for_goal(void) const override;
  bool is_vectoring_to_goal(void) const override;
  bool goal_acquired(void) const override;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_GOAL, /* superstate for finding a goal */
    ST_FINISHED,
    ST_MAX_STATES
  };

  std::shared_ptr<const representation::perceived_arena_map> map(void) const {
    return mc_map;
  }
  void goal_acquired_cb(std::function<bool(bool)> goal_acquired_cb) {
    m_goal_acquired_cb = goal_acquired_cb;
  }
  const fsm::vector_fsm& vector_fsm(void) const { return m_vector_fsm; }
  fsm::vector_fsm& vector_fsm(void) { return m_vector_fsm; }
  const explore_for_goal_fsm& explore_fsm(void) const { return m_explore_fsm; }
  explore_for_goal_fsm& explore_fsm(void) { return m_explore_fsm; }

 private:
  /**
   * @brief Acquire a known goal or discover one via random exploration.
   *
   * @return \c TRUE if a goal has been acquired, \c FALSE otherwise.
   */
  bool acquire_goal(void);

  /**
   * @brief Acquire an unknown goal via exploration.
   *
   * @return \c TRUE if a goal has been acquired \c FALSE otherwise.
   */
  bool acquire_unknown_goal(void);

  /**
   * @brief Acquire a known goal. If the robot's knowledge of the chosen
   * goal's existence expires during the pursuit of said goal, that is
   * ignored.
   */
  virtual bool acquire_known_goal(void) = 0;

  HFSM_STATE_DECLARE_ND(acquire_goal_fsm, start);
  HFSM_STATE_DECLARE_ND(acquire_goal_fsm, fsm_acquire_goal);
  HFSM_STATE_DECLARE_ND(acquire_goal_fsm, finished);

  HFSM_EXIT_DECLARE(acquire_goal_fsm, exit_fsm_acquire_goal);

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
  std::shared_ptr<const representation::perceived_arena_map> mc_map;
  class vector_fsm                                           m_vector_fsm;
  explore_for_goal_fsm                                       m_explore_fsm;
  std::function<bool (bool)>                                 m_goal_acquired_cb;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_GOAL_FSM_HPP_ */
