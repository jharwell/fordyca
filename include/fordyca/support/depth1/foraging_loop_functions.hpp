/**
 * @file foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth0/stateful_foraging_loop_functions.hpp"
#include "fordyca/support/depth1/cache_penalty_handler.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/tasks/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace metrics {
namespace fsm {
class depth1_metrics_collector;
}
namespace tasks {
class execution_metrics_collector;
class management_metrics_collector;
}}

NS_START(support, depth1);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class foraging_loop_functions
 * @ingroup support depth1
 *
 * @brief The loop functions for depth 1 foraging.
 *
 * Handles:
 *
 * - Robots picking up from/dropping in a cache
 * - Subjecting robots using caches to a penalty (only on pickup).
 */
class foraging_loop_functions : public depth0::stateful_foraging_loop_functions {
 public:
  foraging_loop_functions(void);
  ~foraging_loop_functions(void) override;

  foraging_loop_functions(const foraging_loop_functions& s) = delete;
  foraging_loop_functions& operator=(const foraging_loop_functions& s) = delete;

  void Init(argos::TConfigurationNode& node) override;
  void PreStep() override;
  void Destroy(void) override;
  void Reset(void) override;

 protected:
  /**
   * @brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  template<typename T>
  void finish_cached_block_pickup(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    cache_penalty& p = m_cache_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "FATAL: Out of order cache penalty handling");
    ER_ASSERT(controller.cache_acquired(),
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
    int cache_id = utils::robot_on_cache(robot, map());
    if (-1 == cache_id) {
      ER_WARN("WARNING: %s cannot pickup from from cache%d: No such cache",
              controller.GetId().c_str(),
              p.cache_id());
      events::cache_vanished vanished(rcppsw::er::g_server,
                                      p.cache_id());


      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      events::cached_block_pickup pickup_op(rcppsw::er::g_server,
                                            &map()->caches()[p.cache_id()],
                                            utils::robot_id(robot));

      /*
       * Map must be called before controller for proper cache block decrement!
       */
      map()->accept(pickup_op);
      controller.visitor::template visitable_any<T>::accept(pickup_op);
      floor()->SetChanged();
    }
    m_cache_penalty_handler->remove(p);
    ER_ASSERT(!m_cache_penalty_handler->is_serving_penalty<T>(robot),
              "FATAL: Multiple instances of same controller serving cache penalty");
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  template<typename T>
  void finish_cache_block_drop(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    cache_penalty& p = m_cache_penalty_handler->next();
    ER_ASSERT(p.controller() == &controller,
              "FATAL: Out of order cache penalty handling");
    ER_ASSERT(controller.cache_acquired(),
              "FATAL: Controller not waiting for cache block drop");

    /*
     * If a forager enters a cache that only contains 2 blocks on the
     * same/successive/close together timesteps as a collector, then the
     * collector will get a block just fine. The second robot, however, may not
     * be able to drop its block in the cache, depending on if the arena has
     * decided to re-create the static cache yet.
     *
     * This results in a \ref cached_block_drop with a pointer to a cache that
     * has already been destructed, and a segfault. See #247.
     */
    int cache_id = utils::robot_on_cache(robot, map());

    if (-1 == cache_id) {
      ER_WARN("WARNING: %s cannot drop in cache%d: No such cache",
              controller.GetId().c_str(),
              p.cache_id());
      events::cache_vanished vanished(rcppsw::er::g_server,
                                      p.cache_id());

      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      events::cache_block_drop drop_op(rcppsw::er::g_server,
                                       controller.block(),
                                       &map()->caches()[cache_id],
                                       map()->grid_resolution());

      /* Update arena map state due to a cache drop */
      map()->accept(drop_op);
      controller.visitor::template visitable_any<T>::accept(drop_op);
    }
    m_cache_penalty_handler->remove(p);
    ER_ASSERT(!m_cache_penalty_handler->is_serving_penalty<T>(robot),
              "FATAL: Multiple instances of same controller serving cache penalty");
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
  template<typename T>
  bool handle_task_abort(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    if (!controller.task_aborted()) {
      return false;
    }

    /*
     * If a robot aborted its task and was carrying a block it needs to drop it,
     * in addition to updating its own internal state, so that the block is not
     * left dangling and unusable for the rest of the simulation.
     *
     */
    if (controller.is_carrying_block()) {
      ER_NOM("%s aborted task %s while carrying block%d",
             controller.GetId().c_str(),
             controller.current_task()->name().c_str(),
             controller.block()->id());

      /*
       * If the robot is currently right on the edge of a cache, we can't just
       * drop the block here, as it will overlap with the cache, and robots
       * will think that is accessible, but will not be able to vector to it
       * (not all 4 wheel sensors will report the color of a block). See #233.
       */
      bool conflict = false;
      for (auto &cache : map()->caches()) {
        if (block_drop_overlap_with_cache(controller.block(),
                                          cache,
                                          controller.robot_loc())) {
          conflict = true;
        }
      } /* for(cache..) */

      /*
       * If the robot is currently right on the edge of the nest, we can't
       * just drop the block in the nest, as it will not be processed as a
       * normal block_nest_drop, and will be discoverable by a robot via LOS
       * but not able to be acquired, as its color is hidden by that of the
       * nest.
       *
       * If the robot is really close to a wall, then dropping a block may make
       * it inaccessible to future robots trying to reach it, due to obstacle
       * avoidance kicking in. This can result in an endless loop if said block
       * is the only one a robot knows about (see #242).
       */
      if (block_drop_overlap_with_nest(controller.block(),
                                       controller.robot_loc()) ||
          block_drop_near_arena_boundary(controller.block(),
                                         controller.robot_loc())) {
        conflict = true;
      }
      if (!conflict) {
        rcppsw::math::dcoord2 d =
            math::rcoord_to_dcoord(controller.robot_loc(),
                                             map()->grid_resolution());
        events::free_block_drop drop_op(rcppsw::er::g_server,
                                        controller.block(),
                                        d.first,
                                        d.second,
                                        map()->grid_resolution());

        controller.block(nullptr);
        map()->accept(drop_op);
        floor()->SetChanged();
      } else {
        map()->distribute_block(controller.block());
        controller.block(nullptr);
        floor()->SetChanged();
      }
    } else {
      ER_NOM("%s aborted task %s (no block)",
             controller.GetId().c_str(),
             controller.current_task()->name().c_str());
    }
    m_cache_penalty_handler->penalty_abort<decltype(controller)>(robot);
    return true;
  }

  /**
   * @brief Set the LOS of a robot in the arena, INCLUDING handling caches whose
   * extent overlaps the LOS but whose host cell is not in the LOS (see #229).
   */
  template<typename T>
  void set_robot_los(argos::CFootBotEntity& robot,
                     representation::arena_map& map) {
    argos::CVector2 pos;
    pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
            const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    rcppsw::math::dcoord2 robot_loc =
        math::rcoord_to_dcoord(pos, map.grid_resolution());
    auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    std::unique_ptr<representation::line_of_sight> new_los =
        rcppsw::make_unique<representation::line_of_sight>(
            map.subgrid(robot_loc.first, robot_loc.second, 2),
            robot_loc);

    controller.los(new_los);
  }

 private:
  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  bool block_drop_overlap_with_cache(const representation::block* block,
                                     const representation::cache& cache,
                                     const argos::CVector2& drop_loc);
  bool block_drop_overlap_with_nest(const representation::block* block,
                                    const argos::CVector2& drop_loc);
  bool block_drop_near_arena_boundary(const representation::block* block,
                                      const argos::CVector2& drop_loc);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void metric_collecting_init(const struct params::output_params *output_p);
  void cache_handling_init(const struct params::arena_map_params *arenap);
  void handle_arena_interactions(argos::CFootBotEntity &robot);

  // clang-format off
  double                                                       mc_cache_respawn_scale_factor{0.0};
  std::unique_ptr<metrics::fsm::depth1_metrics_collector>      m_depth1_collector;
  std::unique_ptr<metrics::tasks::execution_metrics_collector> m_task_execution_collector;
  std::unique_ptr<metrics::tasks::management_metrics_collector> m_task_management_collector;
  std::shared_ptr<cache_penalty_handler>                       m_cache_penalty_handler;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
