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
#include "fordyca/support/depth1/cache_usage_penalty.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/tasks/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace metrics { namespace collectors {
class task_collector;
namespace fsm {
class depth1_metrics_collector;
}}}
namespace robot_collectors = metrics::collectors::fsm;

NS_START(support, depth1);

class cache_usage_penalty;

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
   * @brief Check if a robot has acquired a cache, and is trying to pickup from
   * a cache, then creates a \ref cache_usage_penalty object and associates it
   * with the robot.
   */
  template<typename T>
  bool init_cache_usage_penalty(
      argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    if (controller.cache_acquired()) {
      /* Check whether the foot-bot is actually on a cache */
      int cache = utils::robot_on_cache(robot, *map());
      if (-1 != cache) {
        ER_ASSERT(!controller.block_detected(),
                  "FATAL: Block detected in cache?");
        ER_NOM("fb%d: start=%u, duration=%u",
               utils::robot_id(robot),
               GetSpace().GetSimulationClock(),
               mc_cache_penalty);
        uint penalty = mc_cache_penalty;
        for (auto it = m_penalty_list.begin(); it != m_penalty_list.end(); ++it) {
          if ((*it)->start_time() == GetSpace().GetSimulationClock()) {
            ++penalty;
            it = m_penalty_list.begin();
          }
        } /* for(i..) */

        m_penalty_list.push_back(new cache_usage_penalty(&controller,
                                                         cache,
                                                         penalty,
                                                         GetSpace().GetSimulationClock()));
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Determine if a robot has satisfied the \ref cache_usage_penalty yet.
   */
  template<typename T>
  bool cache_usage_penalty_satisfied(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_usage_penalty* p) {
                             return p->controller() == &controller;});
    if (it != m_penalty_list.end()) {
      return (*it)->penalty_satisfied(GetSpace().GetSimulationClock());
    }

    return false;
  }

  /**
   * @brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  template<typename T>
  void finish_cached_block_pickup(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    cache_usage_penalty* p = m_penalty_list.front();
    ER_ASSERT(p->controller() == &controller,
              "FATAL: Out of order cache penalty handling");
    ER_ASSERT(controller.cache_acquired(),
              "FATAL: Controller not waiting for cached block pickup");
    events::cached_block_pickup pickup_op(rcppsw::er::g_server,
                                          &map()->caches()[p->cache_id()],
                                          utils::robot_id(robot));
    m_penalty_list.remove(p);
    ER_ASSERT(!robot_serving_cache_penalty<T>(robot),
              "FATAL: Multiple instances of same controller serving cache penalty");

    /*
     * Map must be called before controller for proper cache block decrement!
     */
    map()->accept(pickup_op);
    controller.visitor::template visitable_any<T>::accept(pickup_op);
    floor()->SetChanged();
  }

  /**
   * @brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   */
  template<typename T>
  bool robot_serving_cache_penalty(
      argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_usage_penalty* p) {
                             return p->controller() == &controller;});
    return it != m_penalty_list.end();
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  template<typename T>
  bool handle_cache_block_drop(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    /* Check whether the foot-bot is actually on a cache */
    int cache = utils::robot_on_cache(robot, *map());
    if (-1 != cache) {
      cache_usage_penalty* p = m_penalty_list.front();
      ER_ASSERT(p->controller() == &controller,
                "FATAL: Out of order cache penalty handling");
      ER_ASSERT(controller.cache_acquired(),
                "FATAL: Controller not waiting for cache block drop");

      m_penalty_list.remove(p);
      ER_ASSERT(!robot_serving_cache_penalty<T>(robot),
                "FATAL: Multiple instances of same controller serving cache penalty");

      events::cache_block_drop drop_op(rcppsw::er::g_server,
                                       controller.block(),
                                       &map()->caches()[cache],
                                       map()->grid_resolution());

      /* Update arena map state due to a cache drop */
      map()->accept(drop_op);
      controller.visitor::template visitable_any<T>::accept(drop_op);
      return true;
    }
    return false;
  }

  /**
   * @brief Handle cases in which a robot aborts its current task, and perform
   * any necessary cleanup, such as dropping/distributing a carried block, etc.
   *
   * @param robot The robot to handle task abort for.
   *
   * @return \c TRUE if the robot aborted is current task, \c FALSE otherwise.
   */
  template<typename T>
  bool handle_task_abort(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    /*
     * If a robot aborted its task and was carrying a block it needs to drop it,
     * in addition to updating its own internal state, so that the block is not
     * left dangling and unusable for the rest of the simulation.
     *
     * Also, if the robot happens to abort its task while serving the cache
     * penalty, then it needs to be removed from the penalty list to keep things
     * consistent and avoid assertion failures later.
     */
    if (controller.task_aborted()) {
      if (controller.is_carrying_block()) {
        ER_NOM("%s aborted task %s while carrying block%d",
               controller.GetId().c_str(),
               controller.current_task()->task_name().c_str(),
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
         */
        if (block_drop_overlap_with_nest(controller.block(),
                                         controller.robot_loc())) {
          conflict = true;
        }
        if (!conflict) {
          representation::discrete_coord d =
              representation::real_to_discrete_coord(controller.robot_loc(),
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
               controller.current_task()->task_name().c_str());
      }
      auto it = std::find_if(m_penalty_list.begin(),
                             m_penalty_list.end(),
                             [&](const cache_usage_penalty *p) {
                               return p->controller() == &controller;
                             });
      if (it != m_penalty_list.end()) {
        m_penalty_list.remove(*it);
      }
      ER_ASSERT(
          !robot_serving_cache_penalty<controller::depth1::foraging_controller>(
              robot),
          "FATAL: Multiple instances of same controller serving cache penalty");
      return true;
    }
    return false;
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

    representation::discrete_coord robot_loc =
        representation::real_to_discrete_coord(pos, map.grid_resolution());
    auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    std::unique_ptr<representation::line_of_sight> new_los =
        rcppsw::make_unique<representation::line_of_sight>(
            map.subgrid(robot_loc.first, robot_loc.second, 2),
            robot_loc);

    /* for (auto &c : map.caches()) { */
    /*   argos::CVector2 ll = representation::discrete_to_real_coord( */
    /*       new_los->abs_ll(), */
    /*       map.grid_resolution()); */
    /*   argos::CVector2 lr = representation::discrete_to_real_coord( */
    /*       new_los->abs_lr(), */
    /*       map.grid_resolution()); */
    /*   argos::CVector2 ul = representation::discrete_to_real_coord( */
    /*       new_los->abs_ul(), */
    /*       map.grid_resolution()); */
    /*   argos::CVector2 ur = representation::discrete_to_real_coord( */
    /*       new_los->abs_ur(), */
    /*       map.grid_resolution()); */
    /*   if (c.contains_point(ll) || c.contains_point(lr) || */
    /*       c.contains_point(ul) || c.contains_point(ur)) { */
    /*     auto los_caches = new_los->caches(); */
    /*     new_los->cache_add(&c); */
    /*   } */
    /* } /\* for(&c..) *\/ */

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
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  uint                                                        mc_cache_penalty{0};
  double                                                      mc_cache_respawn_scale_factor{0.0};
  std::unique_ptr<robot_collectors::depth1_metrics_collector> m_depth1_collector;
  std::unique_ptr<metrics::collectors::task_collector>        m_task_collector;
  std::list<cache_usage_penalty*>                             m_penalty_list;
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
