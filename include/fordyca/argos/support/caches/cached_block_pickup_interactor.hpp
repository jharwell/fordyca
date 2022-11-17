/**
 * \file cached_block_pickup_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/cached_block_pickup.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/fsm/cache_acq_validator.hpp"
#include "fordyca/argos/support/caches/base_manager.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/argos/support/tv/cache_op_src.hpp"
#include "fordyca/argos/support/tv/env_dynamics.hpp"
#include "fordyca/argos/support/argos_swarm_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cached_block_pickup_interactor
 * \ingroup argos argos support
 *
 * \brief Handles a robot's (possible) \ref cached_block_pickup event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class cached_block_pickup_interactor : public rer::client<
  cached_block_pickup_interactor<TController, TControllerSpecMap>
  > {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_cache_vanished_visitor_type =
      typename controller_spec::robot_cache_vanished_visitor_type;
  using robot_cached_block_pickup_visitor_type =
      typename controller_spec::robot_cached_block_pickup_visitor_type;

  cached_block_pickup_interactor(carena::caching_arena_map* const map_in,
                                 ::argos::CFloorEntity* const floor_in,
                                 tv::env_dynamics* envd,
                                 fascaches::base_manager* cache_manager,
                                 fasupport::argos_swarm_manager* loop)
      : ER_CLIENT_INIT("fordyca.argos.support.caches.cached_block_pickup_interactor"),
        m_floor(floor_in),
        m_map(map_in),
        m_penalty_handler(
            envd->penalty_handler(tv::cache_op_src::ekEXISTING_CACHE_PICKUP)),
        m_cache_manager(cache_manager),
        m_loop(loop) {}

  cached_block_pickup_interactor(cached_block_pickup_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  cached_block_pickup_interactor(const cached_block_pickup_interactor&) = delete;
  cached_block_pickup_interactor&
  operator=(const cached_block_pickup_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t   The current timestepp.
   */
  fsupport::interactor_status operator()(TController& controller,
                               const rtypes::timestep& t) {
    fsupport::interactor_status status = fsupport::interactor_status::ekNO_EVENT;
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        ER_ASSERT(pre_process_check(controller), "Pre-pickup check failed");
        status = process_cached_block_pickup(controller, t);
        ER_ASSERT(post_process_check(controller), "Post-pickup check failed");
      }
    } else {
      m_penalty_handler->penalty_init(
          controller, tv::cache_op_src::ekEXISTING_CACHE_PICKUP, t);
    }
    return status;
  }

 private:
  /**
   * \brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  fsupport::interactor_status process_cached_block_pickup(TController& controller,
                                                          rtypes::timestep t) {
    const ctv::temporal_penalty& p = m_penalty_handler->penalty_next();
    auto status = fsupport::interactor_status::ekNO_EVENT;

    /*
     * We cannot just lock around the critical arena map updates here in order
     * to make this section thread safe, and need to lock around the whole
     * section below. If two threads updating two robots both having finished
     * serving their penalty this timestep manage to pass the check to actually
     * perform the block pickup before one of them actually finishes picking up
     * a block, then the second one will not get the necessary \ref
     * cache_vanished event. See FORDYCA#594.
     *
     * Grid mutex is also required, but only within the actual \ref
     * cached_block_pickup event visit to the arena map.
     */
    m_map->lock_wr(m_map->cache_mtx());
    m_map->lock_wr(m_map->block_mtx());

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet (for depth 1 simulations).
     *
     * This results in a \ref cached_block_pickup with a pointer to a cache that
     * has already been destructed, and a segfault. See FORDYCA#247.
     *
     * Furthermore, it is also possible that while a robot is serving its pickup
     * penalty that the destination cache disappears AND then is re-created by
     * the arena or another robot dropping a block nearby. This does not appear
     * to be causing an error right now, but very well might in the future, so
     * we check that the ID of the cache we are sitting in/on when we finish
     * serving our penalty is the same as the one the penalty was originally
     * initialized with (not just checking if it is not -1).
     */
    if (p.id() != m_map->robot_on_cache(controller.rpos2D())) {
      ER_WARN("%s cannot pickup from from cache%d: No such cache",
              controller.GetId().c_str(),
              p.id().v());
      m_map->unlock_wr(m_map->block_mtx());
      m_map->unlock_wr(m_map->cache_mtx());

      robot_cache_vanished_visitor_type vanished_op(p.id());
      vanished_op.visit(controller);
    } else {
      fsm::cache_acq_validator v(controller.perception()->known_objects()->known_caches(),
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
      if (v(controller.rpos2D(), p.id(), t)) {
        status = execute_cached_block_pickup(controller, p, t);
        if (status == fsupport::interactor_status::ekCACHE_DEPLETION) {
          m_floor->SetChanged();
        }
      } else {
        ER_WARN("%s cannot pickup from cache%d: Violation of pickup policy",
                controller.GetId().c_str(),
                p.id().v());
      }
      m_map->unlock_wr(m_map->block_mtx());
      m_map->unlock_wr(m_map->cache_mtx());
    }

    m_penalty_handler->penalty_remove(p);

    return status;
  }

  /**
   * \brief Perform the actual pickup of a block from a cache, once all
   * preconditions have been satisfied.
   *
   * Visitation order must be:
   *
   * 1. Arena map (actually remove the cache/ensure proper block decrement)
   * 2. Controller
   * 3. Cache manager (update metrics)
   *
   * No need to lock arena map cache mutex--already holding it from parent
   * function.
   */
  fsupport::interactor_status
  execute_cached_block_pickup(TController& controller,
                              const ctv::temporal_penalty& penalty,
                              const rtypes::timestep& t) {
    auto real_it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == penalty.id(); });
    ER_ASSERT(real_it != m_map->caches().end(),
              "Cache%d from penalty does not exist?",
              penalty.id().v());

    /*
     * Caches have non-owning references to blocks, so even if the current
     * cached block pickup depletes the cache and it is destroyed by the arena
     * map, the reference to the block the robot picked up is still valid,
     * because it points to somewhere within the the block vector owned by the
     * arena map.
     */
    auto* to_pickup = (*real_it)->block_select(m_loop->rng());

    caops::cached_block_pickup_visitor arena_pickup(
        *real_it,
        to_pickup,
        m_loop,
        controller.entity_id(),
        t,
        carena::locking::ekCACHES_HELD | carena::locking::ekBLOCKS_HELD);

    (*real_it)->penalty_served(penalty.penalty());
    controller.block_manip_recorder()->record(
        fmetrics::blocks::block_manip_events::ekCACHE_PICKUP, penalty.penalty());

    auto old_n_caches = m_map->caches().size();

    /* 1st, visit the arena map */
    arena_pickup.visit(*m_map);

    /*
     * The cache has been depleted! The iterator reference we are currently
     * holding to the cache in the arena map caches is now invalid. BUT all is
     * not lost, because the arena map maintains a zombie caches list for
     * exactly this purpose.
     */
    if (m_map->caches().size() < old_n_caches) {
      auto zombie_it =
          std::find_if(m_map->zombie_caches().begin(),
                       m_map->zombie_caches().end(),
                       [&](const auto& c) { return c->id() == penalty.id(); });
      ER_ASSERT(zombie_it != m_map->zombie_caches().end(),
                "Depleted cache%d is not a zombie?",
                penalty.id().v());
      robot_cached_block_pickup_visitor_type robot_zombie_pickup(
          zombie_it->get(), to_pickup, controller.entity_id(), t);

      /* 2nd, visit the controller (depletion case) */
      robot_zombie_pickup.visit(controller);

      /*
       * 3rd, visit the cache manager. This shouldn't be part of the robot event
       * visitor, and can't be part of the arena map visitor, so we do it as its
       * own thing. This may need to be changed in the future...
       */
      std::scoped_lock lock(m_cache_manager->mtx());
      m_cache_manager->cache_depleted(t - (*zombie_it)->creation_ts());
      return fsupport::interactor_status::ekCACHE_DEPLETION;
    } else {
      robot_cached_block_pickup_visitor_type robot_real_pickup(
          *real_it, to_pickup, controller.entity_id(), t);

      /* 2nd, visit the controller (normal case) */
      robot_real_pickup.visit(controller);
      return fsupport::interactor_status::ekNO_EVENT;
    }
  }

  bool pre_process_check(const TController& controller) const {
    const auto& penalty = m_penalty_handler->penalty_next();
    ER_ASSERT(penalty.controller() == &controller,
              "Out of order cache penalty handling");
    const auto* task = dynamic_cast<const events::existing_cache_interactor*>(
        controller.current_task());
    RCPPSW_UNUSED const auto* polled =
        dynamic_cast<const cta::polled_task*>(controller.current_task());
    ER_CHECK(nullptr != task,
             "Non-cache interface task '%s'!",
             polled->name().c_str());
    ER_CHECK(fsm::foraging_acq_goal::ekEXISTING_CACHE ==
                 controller.current_task()->acquisition_goal(),
             "Controller%d@%s/%s not waiting for cached block pickup",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    ER_CHECK(!controller.is_carrying_block(),
             "Controller%d@%s/%s is already carrying block%d",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str(),
             controller.block()->id().v());

    return true;

  error:
    return false;
  }

  bool post_process_check(const TController& controller) const {
    ER_CHECK(!m_penalty_handler->is_serving_penalty(controller),
             "Multiple instances of same controller serving cached block "
             "pickup penalty");

    return true;

  error:
    return false;
  }

 private:
  /* clang-format off */
  ::argos::CFloorEntity* const        m_floor;
  carena::caching_arena_map* const    m_map;
  tv::cache_op_penalty_handler* const m_penalty_handler;
  fascaches::base_manager *           m_cache_manager;
  fasupport::argos_swarm_manager*     m_loop;
  /* clang-format on */
};

NS_END(support, caches, argos, fordyca);

