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

#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/block_proximity.hpp"
#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "rcppsw/math/vector2.hpp"
#include "fordyca/support/tv/tv_manager.hpp"
#include "fordyca/ds/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class cache_site_block_drop_interactor
 * @ingroup fordyca support depth2
 *
 * @brief Handles a robot's (possible) \ref free_block_drop event at a cache
 * site on a given timestep.
 */
template <typename T>
class cache_site_block_drop_interactor : public rer::client<cache_site_block_drop_interactor<T>> {
 public:
  cache_site_block_drop_interactor(ds::arena_map* const map_in,
                                   argos::CFloorEntity* const floor_in,
                                   tv::tv_manager* tv_manager,
                                   dynamic_cache_manager* const cache_manager)
      : ER_CLIENT_INIT("fordyca.support.depth2.cache_site_block_drop_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_cache_manager(cache_manager),
        m_penalty_handler(tv_manager->template penalty_handler<T>(
            tv::block_op_src::ekCACHE_SITE_DROP)) {}

  /**
   * @brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * @todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  cache_site_block_drop_interactor(
      const cache_site_block_drop_interactor& other) = default;
  cache_site_block_drop_interactor& operator=(
      const cache_site_block_drop_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    /*
     * If the controller was serving a penalty and has not finished yet, nothing
     * to do. If the controller was serving a penalty AND has satisfied it as of
     * this timestep, then actually perform the drop.
     */
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->penalty_satisfied(controller, timestep)) {
        finish_cache_site_block_drop(controller);
      }
      return;
    }

    /*
     * If we failed initialize a penalty because there is another block too
     * close, then that probably means that the robot is not aware of said
     * block, so we should send a \ref block_found event to fix that. Better
     * to do this here AND after serving the penalty rather than always just
     * waiting until after the penalty is served to figure out that the robot
     * is too close to a block.
     */
    auto status = m_penalty_handler->penalty_init(controller,
                                                  tv::block_op_src::ekCACHE_SITE_DROP,
                                                  timestep,
                                                  m_cache_manager->cache_proximity_dist(),
                                                  m_cache_manager->block_proximity_dist());
    if (tv::op_filter_status::ekBLOCK_PROXIMITY == status) {
      auto prox_status = loop_utils::cache_site_block_proximity(controller,
                                                                *m_map,
                                                                m_cache_manager->block_proximity_dist());
      ER_ASSERT(-1 != prox_status.entity_id,
                "No block too close with BlockProximity return status");
      block_proximity_notify(controller, prox_status);
    } else if (tv::op_filter_status::ekCACHE_PROXIMITY == status) {
      auto prox_status = loop_utils::new_cache_cache_proximity(controller,
                                                               *m_map,
                                                               m_cache_manager->cache_proximity_dist());
      ER_ASSERT(-1 != prox_status.entity_id,
                "No cache too close with CacheProximity return status");
      cache_proximity_notify(controller, prox_status);
    }
  }

 private:
  void block_proximity_notify(T& controller,
                              const loop_utils::proximity_status_t& status) {
    ER_WARN("%s@%s cannot drop block in cache site: Block%d@%s too close (%f <= %f)",
            controller.GetId().c_str(),
            controller.position().to_str().c_str(),
            status.entity_id,
            status.entity_loc.to_str().c_str(),
            status.distance.length(),
            m_cache_manager->block_proximity_dist());
    events::block_proximity_visitor prox_op(m_map->blocks()[status.entity_id]);
    prox_op.visit(controller);
  }

  void cache_proximity_notify(T& controller,
                              const loop_utils::proximity_status_t& status) {
    ER_WARN("%s@%s cannot drop block in cache site: Cache%d@%s too close (%f <= %f)",
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
    auto it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == status.entity_id; });
    events::cache_proximity_visitor prox_op(*it);
    prox_op.visit(controller);
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache site and is looking to drop an object on it.
   */
  void finish_cache_site_block_drop(T& controller) {
    const tv::temporal_penalty<T>& p = m_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<events::dynamic_cache_interactor*>(
        controller.current_task()), "Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
              tv::acq_goal_type::ekCACHE_SITE == controller.current_task()->acquisition_goal(),
              "Controller not waiting for cache site block drop");
    auto status = loop_utils::cache_site_block_proximity(controller,
                                                             *m_map,
                                                             m_cache_manager->block_proximity_dist());

    /*
     * We checked this before starting to serve a penalty, but it is still
     * possible that another robot dropped a block nearby that we are unaware
     * of, so we need to check again to make sure we can still drop the block on
     * the cache site.
     */
    if (-1 != status.entity_id) {
      block_proximity_notify(controller, status);
    } else {
      perform_cache_site_block_drop(controller, p);
      m_penalty_handler->remove(p);
      ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
                "Multiple instances of same controller serving cache penalty");
    }
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_cache_site_block_drop(T& controller,
                                     const tv::temporal_penalty<T>& penalty) {
    events::free_block_drop_visitor drop_op(m_map->blocks()[penalty.id()],
                                    rmath::dvec2uvec(controller.position(),
                                                     m_map->grid_resolution()),
                                    m_map->grid_resolution());

    drop_op.visit(*m_map);
    drop_op.visit(controller);

    m_floor->SetChanged();
  }

  /* clang-format off */
  argos::CFloorEntity*  const           m_floor;
  ds::arena_map* const                  m_map;
  dynamic_cache_manager*const           m_cache_manager;
  tv::block_op_penalty_handler<T>*const m_penalty_handler;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_SITE_BLOCK_DROP_INTERACTOR_HPP_ */
