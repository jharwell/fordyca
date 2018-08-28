/**
 * @file arena_interactor.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/arena_interactor.hpp"
#include "fordyca/support/cache_op_penalty_handler.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class arena_interactor
 * @ingroup support depth1
 *
 * @brief Handles a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up from/dropping a block in a cache.
 * - Subjecting robots using caches to a penalty on cache drop/pickup.
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 * - Free block drop due to task abort.
 */
template <typename T>
class arena_interactor : public depth0::arena_interactor<T> {
 public:
  arena_interactor(std::shared_ptr<rcppsw::er::server> server,
                   representation::arena_map* const map_in,
                   depth0::stateless_metrics_aggregator *const metrics_agg,
                   argos::CFloorEntity* const floor_in,
                   const ct::waveform_params* const block_manip_penalty,
                   const ct::waveform_params* const cache_usage_penalty)
      : depth0::arena_interactor<T>(server,
                                    map_in,
                                    metrics_agg,
                                    floor_in,
                                    block_manip_penalty),
      m_cache_penalty_handler(server, map_in, cache_usage_penalty) {}

  arena_interactor& operator=(const arena_interactor& other) = delete;
  arena_interactor(const arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (handle_task_abort(controller)) {
        return;
    }
    if (controller.is_carrying_block()) {
      handle_nest_block_drop(controller, timestep);
      if (m_cache_penalty_handler.is_serving_penalty(controller)) {
        if (m_cache_penalty_handler.penalty_satisfied(controller,
                                                      timestep)) {
          finish_cache_block_drop(controller);
        }
      } else {
        m_cache_penalty_handler.penalty_init(controller,
                                             penalty_type::kExistingCacheDrop,
                                             timestep);
      }

    } else { /* The foot-bot has no block item */
      handle_free_block_pickup(controller, timestep);
      if (m_cache_penalty_handler.is_serving_penalty(controller)) {
        if (m_cache_penalty_handler.penalty_satisfied(controller,
                                                      timestep)) {
          finish_cached_block_pickup(controller, timestep);
        }
      } else {
        m_cache_penalty_handler.penalty_init(controller,
                                             penalty_type::kExistingCachePickup,
                                             timestep);
      }
    }
  }

 protected:
  using depth0::arena_interactor<T>::map;
  using depth0::arena_interactor<T>::floor;
  using depth0::arena_interactor<T>::handle_nest_block_drop;
  using depth0::arena_interactor<T>::handle_free_block_pickup;
  using depth0::arena_interactor<T>::nest_drop_penalty_handler;
  using depth0::arena_interactor<T>::free_pickup_penalty_handler;

 private:
  using penalty_type = typename cache_op_penalty_handler<T>::penalty_src;

  /**
   * @brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  void finish_cached_block_pickup(T& controller, uint timestep) {
    const temporal_penalty<T>& p = m_cache_penalty_handler.next();
    ER_ASSERT(p.controller() == &controller,
              "FATAL: Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<tasks::depth1::existing_cache_interactor*>(
        controller.current_task()), "FATAL: Non-cache interface task!");
    ER_ASSERT(acquisition_goal_type::kExistingCache ==
              controller.current_task()->acquisition_goal(),
              "FATAL: Controller not waiting for cached block pickup");

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet.
     *
     * This results in a \ref cached_block_pickup with a pointer to a cache that
     * has already been destructed, and a segfault. See #247.
     */
    if (-1 == utils::robot_on_cache(controller, *map())) {
      ER_WARN("WARNING: %s cannot pickup from from cache%d: No such cache",
              controller.GetId().c_str(),
              p.id());
      events::cache_vanished vanished(depth0::arena_interactor<T>::server_ref(),
                                      p.id());
      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      perform_cached_block_pickup(controller, p, timestep);
      floor()->SetChanged();
    }
    m_cache_penalty_handler.remove(p);
    ER_ASSERT(!m_cache_penalty_handler.is_serving_penalty(controller),
              "FATAL: Multiple instances of same controller serving cache penalty");
  }

  /**
   * @brief Perform the actual pickup of a block from a cache, once all
   * preconditions have been satisfied.
   */
  void perform_cached_block_pickup(T& controller,
                                   const temporal_penalty<T>& penalty,
                                   uint timestep) {
    events::cached_block_pickup pickup_op(rcppsw::er::g_server,
                                          map()->caches()[penalty.id()],
                                          utils::robot_id(controller),
                                          timestep);
    map()->caches()[penalty.id()]->penalty_served(penalty.penalty());

    /*
     * Map must be called before controller for proper cache block decrement!
     */
    map()->accept(pickup_op);
    controller.visitor::template visitable_any<T>::accept(pickup_op);
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  void finish_cache_block_drop(T& controller) {
    const temporal_penalty<T>& p = m_cache_penalty_handler.next();
    ER_ASSERT(p.controller() == &controller,
              "FATAL: Out of order cache penalty handling");
    ER_ASSERT(nullptr != dynamic_cast<tasks::depth1::existing_cache_interactor*>(
        controller.current_task()), "FATAL: Non-cache interface task!");
    ER_ASSERT(controller.current_task()->goal_acquired() &&
              acquisition_goal_type::kExistingCache == controller.current_task()->acquisition_goal(),
              "FATAL: Controller not waiting for cache block drop");

    /*
     * If two collector robots enter a cache that only contains 2 blocks on the
     * same/successive/close together timesteps, then the first robot to serve
     * their penalty will get a block just fine. The second robot, however, may
     * not, depending on if the arena has decided to re-create the static cache
     * yet.
     *
     * This results in a \ref cached_block_drop with a pointer to a cache that
     * has already been destructed, and a segfault. See #247.
     */
    int cache_id = utils::robot_on_cache(controller, *map());

    if (-1 == cache_id) {
      ER_WARN("WARNING: %s cannot drop in cache%d: No such cache",
              controller.GetId().c_str(),
              p.id());
      events::cache_vanished vanished(depth0::arena_interactor<T>::server_ref(),
                                      p.id());

      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      perform_cache_block_drop(controller, p);
    }
    m_cache_penalty_handler.remove(p);
    ER_ASSERT(!m_cache_penalty_handler.is_serving_penalty(controller),
              "FATAL: Multiple instances of same controller serving cache penalty");
  }

  /**
   * @brief Perform the actual dropping of a block in the cache once all
   * preconditions have been satisfied.
   */
  void perform_cache_block_drop(T& controller,
                                const temporal_penalty<T>& penalty) {
    events::cache_block_drop drop_op(rcppsw::er::g_server,
                                     controller.block(),
                                     map()->caches()[penalty.id()],
                                     map()->grid_resolution());
    map()->caches()[penalty.id()]->penalty_served(penalty.penalty());

    /* Update arena map state due to a cache drop */
    map()->accept(drop_op);
    controller.visitor::template visitable_any<T>::accept(drop_op);
  }

  /**
   * @brief Handle cases in which a robot aborts its current task, and perform
   * any necessary cleanup, such as dropping/distributing a carried block, etc.
   *
   * If the robot happens to abort its task while serving the cache penalty,
   * then it is removed from the penalty list to keep things consistent and
   * avoid assertion failures later.
   *
   * @param robot The robot to handle task abort for.
   *
   * @return \c TRUE if the robot aborted is current task, \c FALSE otherwise.
   */
  bool handle_task_abort(T& controller) {

    if (nullptr == controller.current_task() || !controller.task_aborted()) {
      return false;
    }

    /*
     * If a robot aborted its task and was carrying a block, it needs to (1)
     * drop it so that the block is not left dangling and unusable for the rest
     * of the simulation, (2) update its own internal state.
     */
    if (controller.is_carrying_block()) {
      ER_NOM("%s aborted task %s while carrying block%d",
             controller.GetId().c_str(),
             static_cast<tasks::depth1::foraging_task*>(
                 controller.current_task())->name().c_str(),
             controller.block()->id());
      task_abort_with_block(controller);
    } else {
      ER_NOM("%s aborted task %s (no block)",
             controller.GetId().c_str(),
             dynamic_cast<tasks::depth1::foraging_task*>(
                 controller.current_task())->name().c_str());
    }
    if (m_cache_penalty_handler.is_serving_penalty(controller)) {
      m_cache_penalty_handler.penalty_abort(controller);
      ER_NOM("%s aborted task %s while serving cache penalty",
             controller.GetId().c_str(),
             dynamic_cast<tasks::depth1::foraging_task*>(
                 controller.current_task())->name().c_str());
    } else if (free_pickup_penalty_handler().is_serving_penalty(controller)) {
      free_pickup_penalty_handler().penalty_abort(controller);
      ER_NOM("%s aborted task %s while serving free pickup penalty",
             controller.GetId().c_str(),
             dynamic_cast<tasks::depth1::foraging_task*>(
                 controller.current_task())->name().c_str());
    } else if (nest_drop_penalty_handler().is_serving_penalty(controller)) {
      ER_NOM("%s aborted task %s while serving nest drop penalty",
             controller.GetId().c_str(),
             dynamic_cast<tasks::depth1::foraging_task*>(
                 controller.current_task())->name().c_str());
      nest_drop_penalty_handler().penalty_abort(controller);
    }
    return true;
  }

  void task_abort_with_block(T& controller) {
    /*
     * If the robot is currently right on the edge of a cache, we can't just
     * drop the block here, as it will overlap with the cache, and robots
     * will think that is accessible, but will not be able to vector to it
     * (not all 4 wheel sensors will report the color of a block). See #233.
     */
    bool conflict = false;
    for (auto &cache : map()->caches()) {
      if (utils::block_drop_overlap_with_cache(controller.block(),
                                               cache,
                                               controller.robot_loc())) {
        conflict = true;
      }
    } /* for(cache..) */

    /*
     * If the robot is currently right on the edge of the nest, we can't just
     * drop the block in the nest, as it will not be processed as a normal
     * \ref block_nest_drop, and will be discoverable by a robot via LOS but
     * not able to be acquired, as its color is hidden by that of the nest.
     *
     * If the robot is really close to a wall, then dropping a block may make
     * it inaccessible to future robots trying to reach it, due to obstacle
     * avoidance kicking in. This can result in an endless loop if said block
     * is the only one a robot knows about (see #242).
     */
    if (utils::block_drop_overlap_with_nest(controller.block(),
                                            map()->nest(),
                                            controller.robot_loc()) ||
        utils::block_drop_near_arena_boundary(*map(),
                                              controller.block(),
                                              controller.robot_loc())) {
      conflict = true;
    }
    events::free_block_drop drop_op(rcppsw::er::g_server,
                                    controller.block(),
                                    math::rcoord_to_dcoord(controller.robot_loc(),
                                                           map()->grid_resolution()),
                                    map()->grid_resolution());
    if (!conflict) {
      controller.visitor::template visitable_any<T>::accept(drop_op);
      map()->accept(drop_op);
    } else {
      auto b = controller.block();
      map()->distribute_single_block(b);
      controller.visitor::template visitable_any<T>::accept(drop_op);
    }
    floor()->SetChanged();
  }

 private:
  // clang-format off
  cache_op_penalty_handler<T> m_cache_penalty_handler;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_ARENA_INTERACTOR_HPP_ */
