/**
 * @file base_acquire_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include "fordyca/fsm/acquire_goal_fsm.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }
namespace representation { class perceived_arena_map; class cache; }

NS_START(fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_acquire_cache_fsm
 * @ingroup fsm depth1
 *
 * @brief The base FSM for an acquiring from a cache in the arena.
 *
 * Each robot executing this FSM will look for a cache (either a known cache or
 * via random exploration). Once a cache has been acquired, it signals that it
 * has completed its task.
 */
class base_acquire_cache_fsm : public acquire_goal_fsm {
 public:
  base_acquire_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      controller::saa_subsystem* saa,
      representation::perceived_arena_map* map);

  base_acquire_cache_fsm(const base_acquire_cache_fsm& fsm) = delete;
  base_acquire_cache_fsm& operator=(const base_acquire_cache_fsm& fsm) = delete;

 protected:
  /**
   * @brief Get the cache location corresponding to the "best" cache (by some
   * measure), for use in vectoring.
   *
   * @param acquisition The location of the cache to acquire.
   *
   * @return Was the cache selection process successful? (Not guaranteed; we may
   * be too close to our chosen "cache" to vector to it if it is a block).
   */
  virtual bool select_cache_for_acquisition(argos::CVector2* acquisition) = 0;

  argos::CVector2 nest_center(void) const { return mc_nest_center; }

 private:
  bool acquire_known_goal(void) override;
  bool cache_acquired_cb(bool explore_result) const;
  bool cache_detected_cb(void) const;

  // clang-format off
  const argos::CVector2 mc_nest_center;
  // clang-format on
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_ */
