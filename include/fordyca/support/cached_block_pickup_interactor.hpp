/**
 * @file cached_block_pickup_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CACHED_BLOCK_PICKUP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CACHED_BLOCK_PICKUP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/support/cache_op_penalty_handler.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class cached_block_pickup_interactor
 * @ingroup support
 *
 * @brief Handles a robot's (possible) \ref cached_block_pickup event on a given
 * timestep.
 */
template <typename T>
class cached_block_pickup_interactor
    : public er::client<cached_block_pickup_interactor<T>> {
 public:
  cached_block_pickup_interactor(
      ds::arena_map* const map_in,
      argos::CFloorEntity* const floor_in,
      cache_op_penalty_handler<T>* const penalty_handler)
      : ER_CLIENT_INIT("fordyca.support.cached_block_pickup_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_penalty_handler(penalty_handler) {}

  cached_block_pickup_interactor& operator=(
      const cached_block_pickup_interactor& other) = delete;
  cached_block_pickup_interactor(const cached_block_pickup_interactor& other) =
      delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->penalty_satisfied(controller, timestep)) {
        finish_cached_block_pickup(controller, timestep);
      }
    } else {
      m_penalty_handler->penalty_init(controller,
                                      penalty_type::kSrcExistingCachePickup,
                                      timestep);
    }
  }

 private:
  using penalty_type = typename cache_op_penalty_handler<T>::penalty_src;

  /**
   * @brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  void finish_cached_block_pickup(T& controller, uint timestep) {
    const temporal_penalty<T>& p = m_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<tasks::depth1::existing_cache_interactor*>(
                             controller.current_task()),
              "Non-cache interface task!");
    ER_ASSERT(acquisition_goal_type::kExistingCache ==
                  controller.current_task()->acquisition_goal(),
              "Controller not waiting for cached block pickup");

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet.
     *
     * This results in a \ref cached_block_pickup with a pointer to a cache that
     * has already been destructed, and a segfault. See #247.
     *
     * Furthermore, it is also possible that while a robot is serving its pickup
     * penalty that the destination cache disappears AND then is re-created by
     * the arena or another robot dropping a block nearby. This does not appear
     * to be causing an error right now, but very well might in the future, so
     * we check that the ID of the cache we are sitting in/on when we finish
     * serving our penalty is the same as the one the penalty was originally
     * initialized with (not just checking if it is not -1).
     */
    if (p.id() != loop_utils::robot_on_cache(controller, *m_map)) {
      ER_WARN("%s cannot pickup from from cache%d: No such cache",
              controller.GetId().c_str(),
              p.id());
      events::cache_vanished vanished(p.id());
      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      perform_cached_block_pickup(controller, p, timestep);
      m_floor->SetChanged();
    }
    m_penalty_handler->remove(p);
    ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving cache penalty");
  }

  /**
   * @brief Perform the actual pickup of a block from a cache, once all
   * preconditions have been satisfied.
   */
  void perform_cached_block_pickup(T& controller,
                                   const temporal_penalty<T>& penalty,
                                   uint timestep) {
    auto it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == penalty.id(); });
    ER_ASSERT(it != m_map->caches().end(),
              "Cache%d from penalty does not exist",
              penalty.id());
    events::cached_block_pickup pickup_op(*it,
                                          loop_utils::robot_id(controller),
                                          timestep);
    (*it)->penalty_served(penalty.penalty());

    /*
     * Map must be called before controller for proper cache block decrement!
     */
    m_map->accept(pickup_op);
    controller.visitor::template visitable_any<T>::accept(pickup_op);
  }

 private:
  // clang-format off
  argos::CFloorEntity*             const m_floor;
  ds::arena_map* const                   m_map;
  /**
   * @brief Pointer, rather than member, because at most 1 cached block
   * pickup/cache block drop interactions can occur in a single timestep.
   */
  cache_op_penalty_handler<T>*     const m_penalty_handler;
  // clang-format on
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHED_BLOCK_PICKUP_INTERACTOR_HPP_ */
