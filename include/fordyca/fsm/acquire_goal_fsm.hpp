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
#include <functional>
#include <list>
#include <tuple>

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_for_goal_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
namespace rmath = rcppsw::math;
namespace ta = rcppsw::task_allocation;

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
                         public er::client<acquire_goal_fsm>,
                         public metrics::fsm::goal_acquisition_metrics,
                         public ta::taskable {
 public:
  using candidate_type = std::tuple<bool, rmath::vector2d, double>;
  using goal_select_ftype = std::function<candidate_type(void)>;
  using goal_candidates_ftype = std::function<bool(void)>;
  using acquisition_goal_ftype = std::function<acquisition_goal_type(void)>;

  /**
   *
   *
   * @param saa              Handle to sensing and actuation subsystem.
   *
   * @param acquisition_goal Function used to tell the FSM what is the ultimate
   *                         goal of the acquisition. However, the return of
   *                         \ref acquisition_goal() may not always be the same
   *                         as the specified goal, depending on what the
   *                         current FSM state is.
   *
   * @param candidates_exist_cb Function used to determine if any goal
   *                            candidates are currently available/eligible
   *                            for acquisition.
   *
   * @param goal_select      Function used to select a goal from the list of
   *                         candidates. Should return the "best" candidate that
   *                         should be acquired.
   *
   * @param goal_acquired_cb Callback used after a goal has been acquired for
   *                         sanity check/verification of state. Will be passed
   *                         \c TRUE if the acquired goal was obtained via
   *                         exploration, rather than vectoring, and false if it
   *                         was obtained via vectoring. Should return \c TRUE
   *                         if the goal has REALLY been acquired, and \c FALSE
   *                         otherwise (the goal may have vanished if it was a
   *                         block/cache, for example).
   *
   * @param explore_term_cb Callback for goal detection during exploration. This
   *                        fsm can't know when a goal has been reached without
   *                        losing its generality when exploring, so this
   *                        callback is provided to it for that purpose. Should
   *                        return \c TRUE if the goal has been detected/reached
   *                        and exploration should terminate, and \c FALSE
   *                        otherwise.
   */
  acquire_goal_fsm(controller::saa_subsystem* saa,
                   const acquisition_goal_ftype& acquisition_goal,
                   const goal_candidates_ftype& candidates_exist_cb,
                   const goal_select_ftype& goal_select,
                   const std::function<bool(bool)>& goal_acquired_cb,
                   const std::function<bool(void)>& explore_term_cb);
  ~acquire_goal_fsm(void) override = default;

  acquire_goal_fsm(const acquire_goal_fsm& fsm) = delete;
  acquire_goal_fsm& operator=(const acquire_goal_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(const ta::taskable_argument*) override {}
  bool task_finished(void) const override {
    return ST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return ST_ACQUIRE_GOAL == current_state();
  }
  void task_reset(void) override { init(); }

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

  /* goal acquisition metrics */
  bool is_exploring_for_goal(void) const override;
  bool is_vectoring_to_goal(void) const override;
  bool goal_acquired(void) const override;
  acquisition_goal_type acquisition_goal(void) const override;

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

  bool acquire_known_goal(void);

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
  vector_fsm                m_vector_fsm;
  explore_for_goal_fsm      m_explore_fsm;
  acquisition_goal_ftype    m_acquisition_goal;
  goal_candidates_ftype     m_candidates_exist;
  goal_select_ftype         m_goal_select;
  std::function<bool(bool)> m_goal_acquired_cb;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_GOAL_FSM_HPP_ */
