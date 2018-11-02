/**
 * @file cache_site_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/support/block_op_penalty_handler.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/block_proximity.hpp"
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
 * @class cache_site_block_drop_interactor
 * @ingroup support depth2
 *
 * @brief Handles a robot's (possible) \ref free_block_drop event at a cache
 * site on a given timestep.
 */
template <typename T>
class cache_site_block_drop_interactor : public er::client<cache_site_block_drop_interactor<T>> {
 public:
  cache_site_block_drop_interactor(ds::arena_map* const map_in,
                                   argos::CFloorEntity* const floor_in,
                                   const ct::waveform_params* const block_penalty,
                                   dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.cache_site_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_penalty_handler(map_in, block_penalty, "Cache site block drop"),
        m_cache_manager(cache_manager) {}

  cache_site_block_drop_interactor& operator=(
      const cache_site_block_drop_interactor& other) = delete;
  cache_site_block_drop_interactor(
      const cache_site_block_drop_interactor& other) = delete;

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
        finish_cache_site_block_drop(controller);
      }
    } else {
      /*
       * If we failed initialize a penalty because there is another block too
       * close, then that probably means that the robot is not aware of said
       * block, so we should send a \ref block_found event to fix that. Better
       * to do this here AND after serving the penalty rather than always just
       * waiting until after the penalty is served to figure out that the robot
       * is too close to a block.
       */
      penalty_status status = m_penalty_handler.penalty_init(controller,
                                                             penalty_type::kSrcCacheSiteDrop,
                                                             timestep,
                                                             m_cache_manager->cache_proximity_dist(),
                                                             m_cache_manager->block_proximity_dist());
      if (penalty_status::kStatusBlockProximity == status) {
        auto block_pair = loop_utils::cache_site_block_proximity(controller,
                                                                 *m_map,
                                                                 m_cache_manager->block_proximity_dist());
        ER_ASSERT(-1 != block_pair.first,
                  "No block too close with BlockProximity return status");
        block_proximity_notify(controller, block_pair);
      }
    }
  }

 private:
  using penalty_type = typename block_op_penalty_handler<T>::penalty_src;
  using penalty_status = typename block_op_penalty_handler<T>::penalty_status;

  void block_proximity_notify(T& controller, std::pair<int, argos::CVector2> block_pair) {
    ER_WARN("%s cannot drop block in cache site (%f, %f): Block%d too close (%f <= %f)",
            controller.GetId().c_str(),
            controller.robot_loc().GetX(),
            controller.robot_loc().GetY(),
            block_pair.first,
            block_pair.second.Length(),
            m_cache_manager->block_proximity_dist());
    events::block_proximity prox(m_map->blocks()[block_pair.first]->clone());
    controller.visitor::template visitable_any<T>::accept(prox);
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  void finish_cache_site_block_drop(T& controller) {
    const temporal_penalty<T>& p = m_penalty_handler.next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<tasks::depth2::dynamic_cache_interactor*>(
        controller.current_task()), "Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
              acquisition_goal_type::kCacheSite == controller.current_task()->acquisition_goal(),
              "Controller not waiting for cache site block drop");
    auto block_pair = loop_utils::cache_site_block_proximity(controller,
                                                             *m_map,
                                                             m_cache_manager->block_proximity_dist());

    /*
     * We checked this before starting to serve a penalty, but it is still
     * possible that another robot dropped a block nearby that we are unaware
     * of, so we need to check again to make sure we can still drop the block on
     * the cache site.
     */
    if (-1 != block_pair.first) {
      block_proximity_notify(controller, block_pair);
    } else {
      perform_cache_site_block_drop(controller, p);
      m_penalty_handler.remove(p);
      ER_ASSERT(!m_penalty_handler.is_serving_penalty(controller),
                "Multiple instances of same controller serving cache penalty");
    }
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_cache_site_block_drop(T& controller,
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

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_ */
