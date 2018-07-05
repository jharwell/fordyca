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
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }

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
class acquire_cache_site_fsm : public acquire_goal_fsm {
 public:
  acquire_cache_site_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      std::shared_ptr<const representation::perceived_arena_map> map);

  acquire_cache_site_fsm(const acquire_cache_site_fsm& fsm) = delete;
  acquire_cache_site_fsm& operator=(const acquire_cache_site_fsm& fsm) = delete;

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;

 private:
  bool acquire_known_goal(void) override;
  bool site_acquired_cb(bool explore_result) const;

  /**
   * @brief Callback used by the \ref acquire_goal_fsm to determine if the goal
   * has actually been acquired. For this class, that means that the site is
   * available/there is not another cache nearby. For now, those concerns are
   * ignored, so that once the robot arrives at the chosen site, it can just
   * signal that it has acquired its goal.
   */
  bool site_detected_cb(void) const { return true; }

  // clang-format off
  const argos::CVector2 mc_nest_center;
  // clang-format on
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_ACQUIRE_CACHE_SITE_FSM_HPP_ */
