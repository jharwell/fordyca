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

#include "fordyca/support/tv/tv_manager.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/ds/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class new_cache_block_drop_interactor
 * @ingroup fordyca support depth2
 *
 * @brief Handles a robot's (possible) \ref free_block_drop event at a new cache
 * on a given timestep.
 */
template <typename T>
class new_cache_block_drop_interactor : public er::client<new_cache_block_drop_interactor<T>> {
 public:
  new_cache_block_drop_interactor(ds::arena_map* const map_in,
                                   argos::CFloorEntity* const floor_in,
                                  tv::tv_manager* const tv_manager,
                                   dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.new_cache_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_cache_manager(cache_manager),
        m_penalty_handler(tv_manager->template penalty_handler<T>(
            tv::block_op_src::kSrcNewCacheDrop)) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  new_cache_block_drop_interactor(
      const new_cache_block_drop_interactor& other) = default;
  new_cache_block_drop_interactor& operator=(
      const new_cache_block_drop_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   *
   * @return \c TRUE if a block was dropped in a new cache, \c FALSE otherwise.
   */
  bool operator()(T& controller, uint timestep) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->penalty_satisfied(controller, timestep)) {
        return finish_new_cache_block_drop(controller);
      }
    } else {
      /*
       * If we failed initialize a penalty because there is another
       * block/cache too close, then that probably means that the robot is not
       * aware of said block, so we should send an event to fix that. Better
       * to do this here AND after serving the penalty rather than always just
       * waiting until after the penalty is served to figure out that the
       * robot is too close to a block/cache.
       */
      penalty_status status = m_penalty_handler->penalty_init(controller,
                                                   tv::block_op_src::kSrcNewCacheDrop,
                                                   timestep,
                                                   m_cache_manager->cache_proximity_dist(),
                                                   m_cache_manager->block_proximity_dist());
      if (penalty_status::kStatusCacheProximity == status) {
        auto prox_status = loop_utils::new_cache_cache_proximity(controller,
                                                                 *m_map,
                                                                 m_cache_manager->cache_proximity_dist());
        ER_ASSERT(-1 != prox_status.entity_id,
                  "No cache too close with CacheProximity return status");
        cache_proximity_notify(controller, prox_status);
      }
    }
    return false;
  }

 private:
  using penalty_status = typename tv::tv_manager::filter_status<T>;

  void cache_proximity_notify(T& controller,
                              const loop_utils::proximity_status_t& status) {
    ER_WARN("%s@%s cannot drop block in new cache: Cache%d@%s too close (%f <= %f)",
            controller.GetId().c_str(),
            controller.position().to_str().c_str(),
            status.entity_id,
            status.entity_loc.to_str().c_str(),
            status.distance.length(),
            m_cache_manager->cache_proximity_dist());
    /*
     * Because caches can be dynamically created/destroyed, we cannot rely on
     * the index position of cache i to be the same as its ID, so we need to
     * search for the correct cache.
     */
    auto it = std::find_if(m_map->caches().begin(),
                           m_map->caches().end(),
                           [&](const auto& c) {
                             return c->id() == status.entity_id;
                           });
    ER_ASSERT(m_map->caches().end() != it,
              "FATAL: Cache%d does not exist?",
              status.entity_id);
    events::cache_proximity_visitor prox_op(*it);
    prox_op.visit(controller);
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  bool finish_new_cache_block_drop(T& controller) {
    const tv::temporal_penalty<T>& p = m_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<events::dynamic_cache_interactor*>(
        controller.current_task()), "Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
              tv::acquisition_goal_type::kNewCache == controller.current_task()->acquisition_goal(),
              "Controller not waiting for new cache block drop");
    auto status = loop_utils::new_cache_cache_proximity(controller,
                                                       *m_map,
                                                       m_cache_manager->cache_proximity_dist());

    if (-1 != status.entity_id) {
      /*
     * If there is another cache nearby that the robot is unaware of, and if
     * that cache is close enough to the robot's current location that a block
     * drop would result in the creation of a new cache which would overlap with
     * said cache, then abort the drop and tell the robot about the undiscovered
     * cache so that it will update its state and pick a different new cache.
     */
      ER_WARN("%s cannot drop block in new cache %s: Cache%d too close (%f <= %f)",
              controller.GetId().c_str(),
              controller.position().to_str().c_str(),
              status.entity_id,
              status.distance.length(),
              m_cache_manager->cache_proximity_dist());

      /*
       * We need to perform the proxmity check again after serving our block
       * drop penalty, because a cache might have been created nearby while we
       * were waiting, rendering our chosen location invalid.
       */
      auto it = std::find_if(m_map->caches().begin(),
                             m_map->caches().end(),
                             [&](const auto& c) {
                               return c->id() == status.entity_id; });
      ER_ASSERT(m_map->caches().end() != it,
                "FATAL: Cache%d does not exist?",
                status.entity_id);
      events::cache_proximity_visitor prox_op(*it);
      prox_op.visit(controller);
      return false;
    } else {
      perform_new_cache_block_drop(controller, p);
      m_penalty_handler->remove(p);
      ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
                "Multiple instances of same controller serving cache penalty");
      return true;
    }
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_new_cache_block_drop(T& controller,
                                    const tv::temporal_penalty<T>& penalty) {
    events::free_block_drop_visitor drop_op(m_map->blocks()[penalty.id()],
                                            rmath::dvec2uvec(controller.position(),
                                                             m_map->grid_resolution()),
                                            m_map->grid_resolution());

    drop_op.visit(controller);
    drop_op.visit(*m_map);
    m_floor->SetChanged();
  }

  /* clang-format off */
  argos::CFloorEntity*  const            m_floor;
  ds::arena_map* const                   m_map;
  dynamic_cache_manager*const            m_cache_manager;
  tv::block_op_penalty_handler<T>* const m_penalty_handler;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_NEW_CACHE_BLOCK_DROP_INTERACTOR_HPP_ */
