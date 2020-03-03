/**
 * \file cached_block_pickup_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "cosm/foraging/ds/arena_map.hpp"
#include "cosm/foraging/events/arena_cached_block_pickup.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"

#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/events/robot_cached_block_pickup.hpp"
#include "fordyca/fsm/cache_acq_validator.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cached_block_pickup_interactor
 * \ingroup support
 *
 * \brief Handles a robot's (possible) \ref cached_block_pickup event on a given
 * timestep.
 */
template <typename T>
class cached_block_pickup_interactor
    : public rer::client<cached_block_pickup_interactor<T>> {
 public:
  cached_block_pickup_interactor(cfds::arena_map* const map_in,
                                 argos::CFloorEntity* const floor_in,
                                 tv::env_dynamics* envd,
                                 support::base_cache_manager* cache_manager,
                                 support::base_loop_functions* loop)
      : ER_CLIENT_INIT("fordyca.support.cached_block_pickup_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_penalty_handler(
            envd->penalty_handler(tv::cache_op_src::ekEXISTING_CACHE_PICKUP)),
        m_cache_manager(cache_manager),
        m_loop(loop) {}

  /**
   * \brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * \todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  cached_block_pickup_interactor(const cached_block_pickup_interactor& other) =
      default;
  cached_block_pickup_interactor& operator=(
      const cached_block_pickup_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t   The current timestepp.
   */
  interactor_status operator()(T& controller, const rtypes::timestep& t) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        return finish_cached_block_pickup(controller, t);
      }
    } else {
      m_penalty_handler->penalty_init(controller,
                                      tv::cache_op_src::ekEXISTING_CACHE_PICKUP,
                                      t);
    }
    return interactor_status::ekNO_EVENT;
  }

 private:
  /**
   * \brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  interactor_status finish_cached_block_pickup(T& controller,
                                               rtypes::timestep t) {
    const tv::temporal_penalty& p = m_penalty_handler->penalty_next();
    ER_ASSERT(p.controller() == &controller,
              "Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<events::existing_cache_interactor*>(
                             controller.current_task()),
              "Non-cache interface task!");
    ER_ASSERT(fsm::foraging_acq_goal::type::ekEXISTING_CACHE ==
                  controller.current_task()->acquisition_goal(),
              "Controller not waiting for cached block pickup");
    ER_ASSERT(!controller.is_carrying_block(),
              "Controller is already carrying block%d",
              controller.block()->id().v());
    /*
     * We cannot just lock around the critical arena map updates here in order
     * to make this section thread safe, and need to lock around the whole
     * section below. If two threads updating two robots both having finished
     * serving their penalty this timestep manage to pass the check to actually
     * perform the block pickup before one of them actually finishes picking up
     * a block, then the second one will not get the necessary \ref
     * cache_vanished event. See #594.
     *
     * Grid and block mutexes are also required, but only within the actual \ref
     * cached_block_pickup event visit to the arena map.
     */
    m_map->cache_mtx()->lock();

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet (for depth 1 simulations).
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
    auto status = interactor_status::ekNO_EVENT;
    if (p.id() != utils::robot_on_cache(controller, *m_map)) {
      ER_WARN("%s cannot pickup from from cache%d: No such cache",
              controller.GetId().c_str(),
              p.id().v());
      events::cache_vanished_visitor vanished_op(p.id());
      vanished_op.visit(controller);
    } else {
      fsm::cache_acq_validator v(&controller.perception()->dpo_store()->caches(),
                                 controller.cache_sel_matrix(),
                                 true);
      /*
       * If the cache still exists after a robot serves its penalty we still
       * need to double check that it does not violate the robot's cache pickup
       * policy, because it is possible that two robots entering a cache on the
       * same/successive/close together timesteps to both be able to start
       * serving their respective penalties (and not violate their respective
       * pickup policies), but that the one who picks up a block first would
       * cause the second one to violate THEIR pickup policy if they picked up a
       * block (e.g. the cache now has too few blocks for pickup).
       *
       * In this case, you don't need to do anything, as the change in the
       * cache's status will be picked up by the second robot next timestep.
       */
      if (v(controller.position2D(), p.id(), t)) {
        status = perform_cached_block_pickup(controller, p, t);
        m_floor->SetChanged();
      } else {
        ER_WARN("%s cannot pickup from cache%d: Violation of pickup policy",
                controller.GetId().c_str(),
                p.id().v());
      }
    }
    m_map->cache_mtx()->unlock();

    m_penalty_handler->penalty_remove(p);
    ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving cache penalty");
    return status;
  }

  /**
   * \brief Perform the actual pickup of a block from a cache, once all
   * preconditions have been satisfied.
   */
  interactor_status perform_cached_block_pickup(
      T& controller,
      const tv::temporal_penalty& penalty,
      rtypes::timestep t) {
    auto it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == penalty.id(); });
    ER_ASSERT(it != m_map->caches().end(),
              "Cache%d from penalty does not exist?",
              penalty.id().v());
    cfevents::arena_cached_block_pickup_visitor apickup_op(
        *it, m_loop, controller.entity_id(), t);
    const crepr::base_block2D* to_pickup = (*it)->oldest_block();
    events::robot_cached_block_pickup_visitor rpickup_op(
        *it, to_pickup, controller.entity_id(), t);
    (*it)->penalty_served(penalty.penalty());

    uint old_n_caches = m_map->caches().size();

    /*
     * Visitation order must be:
     *
     * 1. Cache manager (update metrics)
     * 2. Arena map (actually remove the cache/ensure proper block decrement)
     * 3. Controller
     *
     * No need to lock arena map cache mutex--already holding it from parent
     * function.
     */
    rpickup_op.visit(*m_cache_manager);
    apickup_op.visit(*m_map);
    rpickup_op.visit(controller);

    if (m_map->caches().size() < old_n_caches) {
      return interactor_status::ekCACHE_DEPLETION;
    }
    return interactor_status::ekNO_EVENT;
  }

 private:
  /* clang-format off */
  argos::CFloorEntity* const          m_floor;
  cfds::arena_map* const              m_map;
  tv::cache_op_penalty_handler* const m_penalty_handler;
  base_cache_manager *                m_cache_manager;
  base_loop_functions*                m_loop;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHED_BLOCK_PICKUP_INTERACTOR_HPP_ */
