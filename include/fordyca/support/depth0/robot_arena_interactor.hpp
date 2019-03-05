/**
 * @file robot_arena_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/free_block_pickup_interactor.hpp"
#include "fordyca/support/nest_block_drop_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class robot_arena_interactor
 * @ingroup support depth0
 *
 * @brief Handle's a robot's interactions with the environment on each timestep:
 *
 * - Picking up a free block (possibly with penalty).
 * - Dropping a carried block in the nest (possibly with a penalty).
 */
template <typename T>
class robot_arena_interactor : public er::client<robot_arena_interactor<T>> {
 public:
  using controller_type = T;

  robot_arena_interactor(ds::arena_map* const map,
                         depth0_metrics_aggregator *const metrics_agg,
                         argos::CFloorEntity* const floor,
                         tv::tv_manager* const tv_manager)
      : ER_CLIENT_INIT("fordyca.support.depth0.robot_arena_interactor"),
        m_free_pickup_interactor(map, floor, tv_manager),
        m_nest_drop_interactor(map, metrics_agg, floor, tv_manager) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  robot_arena_interactor(const robot_arena_interactor& other) = default;
  robot_arena_interactor& operator=(const robot_arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for the interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep The current timestep.
   */
  template<typename C = T>
  void operator()(C& controller, uint timestep) {
    if (controller.is_carrying_block()) {
      m_nest_drop_interactor(controller, timestep);
    } else { /* The foot-bot has no block item */
      m_free_pickup_interactor(controller, timestep);
    }
  }

 private:
  /* clang-format off */
  free_block_pickup_interactor<T> m_free_pickup_interactor;
  nest_block_drop_interactor<T>   m_nest_drop_interactor;
  /* clang-format on */
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ROBOT_ARENA_INTERACTOR_HPP_ */
