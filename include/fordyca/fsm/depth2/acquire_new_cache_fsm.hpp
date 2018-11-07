/**
 * @file acquire_new_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_NEW_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_NEW_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { class cache_sel_matrix; }
namespace representation { class perceived_arena_map; }

NS_START(fsm, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_new_cache_fsm
 * @ingroup fsm depth2
 *
 * @brief The FSM for an acquiring a NEW cache within the arena.
 *
 * Each robot executing this FSM will look for a new cache (either a known new
 * cache or via random exploration). Once the chosen new cache has been
 * acquired, it signals that it has completed its task.
 */
class acquire_new_cache_fsm : public acquire_goal_fsm,
                              public er::client<acquire_new_cache_fsm> {
 public:
  acquire_new_cache_fsm(
      const controller::cache_sel_matrix* csel_matrix,
      controller::saa_subsystem* actuators,
      ds::perceived_arena_map* map);

  acquire_new_cache_fsm(const acquire_new_cache_fsm& fsm) = delete;
  acquire_new_cache_fsm& operator=(const acquire_new_cache_fsm& fsm) = delete;

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;

 private:
  bool select_cache_for_acquisition(argos::CVector2 * acquisition);
  bool acquire_known_goal(void) override;
  bool cache_acquired_cb(bool explore_result) const;

  // clang-format off
  const controller::cache_sel_matrix* mc_sel_matrix;
  // clang-format on
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_NEW_CACHE_FSM_HPP_ */
