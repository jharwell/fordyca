/**
 * @file acquire_cache_site_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_CACHE_SITE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_CACHE_SITE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { class cache_sel_matrix; }

NS_START(fsm, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_cache_site_fsm
 * @ingroup fsm depth2
 *
 * @brief The FSM for acquiring a site to start a new cache at within the
 * arena.
 *
 * Each robot executing this FSM will compute the "best" site to start a new
 * cache at, and then acquire that location within the arena. Once this has been
 * done, it signals that it has completed its task.
 *
 * @bug There is not currently any handling for what needs to happen if the
 * robot arrives at its chosen site and there is already a cache there or there
 * is one very close by.
 */
class acquire_cache_site_fsm : public acquire_goal_fsm,
                               public er::client<acquire_cache_site_fsm> {
 public:
  acquire_cache_site_fsm(
      const controller::cache_sel_matrix* csel_matrix,
      controller::saa_subsystem* saa,
      ds::perceived_arena_map* map);

  acquire_cache_site_fsm(const acquire_cache_site_fsm& fsm) = delete;
  acquire_cache_site_fsm& operator=(const acquire_cache_site_fsm& fsm) = delete;

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;

 private:
  bool acquire_known_goal(void) override;
  bool site_acquired_cb(bool explore_result) const;

  // clang-format off
  const controller::cache_sel_matrix* const mc_matrix;
  // clang-format on
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_CACHE_SITE_FSM_HPP_ */
