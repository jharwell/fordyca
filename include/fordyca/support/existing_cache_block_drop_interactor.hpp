/**
 * \file existing_cache_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/cache_block_drop.hpp"

#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class existing_cache_block_drop_interactor
 * \ingroup support
 *
 * \brief Handles a robot's (possible) \ref cache_block_drop event for existing
 * caches on a given timestep.
 */
template <typename TController, typename TControllerSpecMap>
class existing_cache_block_drop_interactor
    : public rer::client<
  existing_cache_block_drop_interactor<TController, TControllerSpecMap>
  > {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_cache_vanished_visitor_type =
      typename controller_spec::robot_cache_vanished_visitor_type;
  using robot_cache_block_drop_visitor_type =
      typename controller_spec::robot_cache_block_drop_visitor_type;

  existing_cache_block_drop_interactor(carena::caching_arena_map* const map_in,
                                       tv::env_dynamics* envd)
      : ER_CLIENT_INIT("fordyca.support.existing_cache_block_drop_interactor"),
        m_map(map_in),
        m_penalty_handler(
            envd->penalty_handler(tv::cache_op_src::ekEXISTING_CACHE_DROP)) {}

  existing_cache_block_drop_interactor(existing_cache_block_drop_interactor&&) =
      default;

  /* Not copy-constructible/assignable by default. */
  existing_cache_block_drop_interactor(
      const existing_cache_block_drop_interactor&) = delete;
  existing_cache_block_drop_interactor&
  operator=(const existing_cache_block_drop_interactor&) = delete;

  /**
   * \brief The actual handling function for interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t   The current timestep.
   */
  void operator()(TController& controller, rtypes::timestep t) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        ER_ASSERT(pre_process_check(controller), "Pre-drop check failed");
        process_cache_block_drop(controller);
        ER_ASSERT(post_process_check(controller), "Post-drop check failed");
      }
    } else {
      m_penalty_handler->penalty_init(
          controller, tv::cache_op_src::ekEXISTING_CACHE_DROP, t);
    }
  }

 private:
  /**
   * \brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  void process_cache_block_drop(TController& controller) {
    const auto& penalty = m_penalty_handler->penalty_next();
    /*
     * We cannot just lock around the critical arena map updates here in order
     * to make this section thread safe, and need to lock around the whole
     * section below. If two threads updating two robots both having finished
     * serving their penalty this timestep manage to pass the check to actually
     * perform the cache pickup before one of them actually finishes picking up
     * a block, then the second one will not get the necessary \ref
     * cache_vanished event. See FORDYCA#594.
     *
     * Grid and block mutexes are also required, but only within the actual \ref
     * cached_block_pickup event visit to the arena map.
     */
    m_map->lock_wr(m_map->cache_mtx());
    m_map->lock_rd(m_map->block_mtx());

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet.
     *
     * This results in a \ref cached_block_drop with a pointer to a cache that
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
    auto cache_id = m_map->robot_on_cache(controller.rpos2D());

    if (penalty.id() != cache_id) {
      ER_WARN("%s cannot drop in cache%d: No such cache",
              controller.GetId().c_str(),
              penalty.id().v());
      /*
       * We now know we aren't going to update arena state, because the cache
       * associated with the penalty doesn't exist anymore.
       */
      m_map->unlock_rd(m_map->block_mtx());
      m_map->unlock_wr(m_map->cache_mtx());

      robot_cache_vanished_visitor_type vanished_op(penalty.id());
      vanished_op.visit(controller);
    } else {
      execute_cache_block_drop(controller, penalty);
      m_map->unlock_rd(m_map->block_mtx());
      m_map->unlock_wr(m_map->cache_mtx());
    }

    m_penalty_handler->penalty_remove(penalty);
  }

  /**
   * \brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void execute_cache_block_drop(TController& controller,
                                const ctv::temporal_penalty& penalty) {
    auto cache_it =
        std::find_if(m_map->caches().begin(),
                     m_map->caches().end(),
                     [&](const auto& c) { return c->id() == penalty.id(); });
    ER_ASSERT(cache_it != m_map->caches().end(),
              "Cache%d from penalty does not exist",
              penalty.id().v());

    rtypes::type_uuid block_id = controller.block()->id();

    /*
     * Safe to directly index into arena map block vector without locking
     * because the blocks never move from their original locations.
     *
     * Need to tell event to perform \ref arena_map block locking because there
     * we are only holding the cache mutex.
     */
    caops::cache_block_drop_visitor adrop_op(
        m_map->blocks()[block_id.v()],
        *cache_it,
        m_map->grid_resolution(),
        carena::locking::ekCACHES_HELD | carena::locking::ekBLOCKS_HELD);
    robot_cache_block_drop_visitor_type rdrop_op(controller.block_release(),
                                                 *cache_it,
                                                 m_map->grid_resolution());

    (*cache_it)->penalty_served(penalty.penalty());
    controller.block_manip_recorder()->record(
        metrics::blocks::block_manip_events::ekCACHE_DROP, penalty.penalty());
    /*
     * Order of visitation must be:
     *
     * 1. Arena map
     * 2. Controller
     *
     * In order for proper \ref events::cache_block_drop processing.
     */
    adrop_op.visit(*m_map);
    rdrop_op.visit(controller);
  }

  bool pre_process_check(const TController& controller) const {
    const auto& penalty = m_penalty_handler->penalty_next();
    auto acq_goal = controller.current_task()->acquisition_goal();
    const auto* task = dynamic_cast<const events::existing_cache_interactor*>(
        controller.current_task());
    RCPPSW_UNUSED const auto* polled =
        dynamic_cast<const cta::polled_task*>(controller.current_task());

    ER_CHECK(penalty.controller() == &controller,
             "Out of order cache penalty handling");
    ER_CHECK(nullptr != task,
             "Non-cache interface task '%s'!",
             polled->name().c_str());
    ER_CHECK(controller.current_task()->goal_acquired() &&
                 fsm::foraging_acq_goal::ekEXISTING_CACHE == acq_goal,
             "Robot%d@%s/%s not waiting for cache block drop",
             controller.entity_id().v(),
             rcppsw::to_string(controller.rpos2D()).c_str(),
             rcppsw::to_string(controller.dpos2D()).c_str());
    return true;

  error:
    return false;
  }

  bool post_process_check(const TController& controller) const {
    ER_CHECK(!m_penalty_handler->is_serving_penalty(controller),
             "Multiple instances of same controller serving existing cache "
             "drop penalty");

    return true;

  error:
    return false;
  }
  /* clang-format off */
  carena::caching_arena_map* const   m_map;
  tv::cache_op_penalty_handler*const m_penalty_handler;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_EXISTING_CACHE_BLOCK_DROP_INTERACTOR_HPP_ */
