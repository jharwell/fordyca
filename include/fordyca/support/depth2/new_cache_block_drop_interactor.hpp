/**
 * @file new_cache_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/support/block_op_penalty_handler.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/tasks/depth2/dynamic_cache_interactor.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class new_cache_block_drop_interactor
 * @ingroup support depth2
 *
 * @brief Handles a robot's (possible) \ref free_block_drop event at a new cache
 * on a given timestep.
 */
template <typename T>
class new_cache_block_drop_interactor : public er::client<new_cache_block_drop_interactor<T>> {
 public:
  new_cache_block_drop_interactor(ds::arena_map* const map_in,
                                   argos::CFloorEntity* const floor_in,
                                   const ct::waveform_params* const block_penalty,
                                   dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.new_cache_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_penalty_handler(map_in, block_penalty, "New cache block drop"),
        m_cache_manager(cache_manager) {}

  new_cache_block_drop_interactor& operator=(
      const new_cache_block_drop_interactor& other) = delete;
  new_cache_block_drop_interactor(
      const new_cache_block_drop_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (m_penalty_handler.is_serving_penalty(controller)) {
      if (m_penalty_handler.penalty_satisfied(controller,
                                              timestep)) {
        finish_new_cache_block_drop(controller);
      }
    } else {
      m_penalty_handler.penalty_init(controller,
                                     penalty_type::kNewCacheDrop,
                                     timestep);
    }
  }

 private:
  using penalty_type = typename block_op_penalty_handler<T>::penalty_src;

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  void finish_new_cache_block_drop(T& controller) {
    const temporal_penalty<T>& p = m_penalty_handler.next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<tasks::depth2::dynamic_cache_interactor*>(
        controller.current_task()), "Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
              acquisition_goal_type::kCacheSite == controller.current_task()->acquisition_goal(),
              "Controller not waiting for cache site block drop");
    auto cache_pair = loop_utils::new_cache_cache_proximity(controller,
                                                       *m_map,
                                                       m_cache_manager->cache_proximity_dist());

    if (-1 != cache_pair.first) {
      /*
     * If there is another cache nearby that the robot is unaware of, and if
     * that cache is close enough to the robot's current location that a block
     * drop would result in the creation of a new cache which would overlap with
     * said cache, then abort the drop and tell the robot about the undiscovered
     * cache so that it will update its state and pick a different new cache.
     */
      ER_WARN("%s cannot drop block in cache site (%f, %f): Cache%d too close (%f <= %f)",
              controller.GetId().c_str(),
              controller.robot_loc().GetX(),
              controller.robot_loc().GetY(),
              cache_pair.first,
              cache_pair.second.Length(),
              m_cache_manager->cache_proximity_dist());
      events::cache_found prox(m_map->caches()[cache_pair.first]);
      controller.visitor::template visitable_any<T>::accept(prox);
    } else {
      perform_new_cache_block_drop(controller, p);
      m_penalty_handler.remove(p);
      ER_ASSERT(!m_penalty_handler.is_serving_penalty(controller),
                "Multiple instances of same controller serving cache penalty");
    }
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_new_cache_block_drop(T& controller,
                                     const temporal_penalty<T>& penalty) {
    events::free_block_drop drop_op(m_map->blocks()[penalty.id()],
                                    math::rcoord_to_dcoord(controller.robot_loc(),
                                                           m_map->grid_resolution()),
                                    m_map->grid_resolution());

    /* Update arena map state due to a free block drop */
    m_map->accept(drop_op);
    controller.visitor::template visitable_any<T>::accept(drop_op);
    m_floor->SetChanged();
  }

  // clang-format off
  argos::CFloorEntity*  const m_floor;
  ds::arena_map* const        m_map;
  block_op_penalty_handler<T> m_penalty_handler;
  dynamic_cache_manager*const m_cache_manager;
  // clang-format on
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_ */
