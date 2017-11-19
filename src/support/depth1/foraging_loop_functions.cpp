/**
 * @file foraging_loop_functions.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/foraging_loop_functions.hpp"
#include <limits>
#include <random>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/metrics/collectors/robot_metrics/depth1_collector.hpp"
#include "fordyca/metrics/collectors/task_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/depth0_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/random_metrics_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/distance_metrics_collector.hpp"
#include "fordyca/support/depth1/cache_usage_penalty.hpp"
#include "fordyca/expressions/cache_respawn_probability.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

namespace robot_collectors = metrics::collectors::robot_metrics;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void) :
    mc_cache_penalty(), mc_cache_respawn_scale_factor(), m_depth1_collector(),
    m_task_collector(), m_penalty_list() {}

foraging_loop_functions::~foraging_loop_functions(void) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  depth0::foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1_foraging loop functions");
  params::loop_function_repository repo;

  repo.parse_all(node);

  /* initialize stat collecting */
  mc_cache_penalty = static_cast<const struct params::loop_functions_params*>(
      repo.get_params("loop_functions"))->cache.usage_penalty;
  mc_cache_respawn_scale_factor = static_cast<const struct params::loop_functions_params*>(
      repo.get_params("loop_functions"))->cache.static_respawn_scale_factor;

  m_depth1_collector.reset(new robot_collectors::depth1_collector(
      static_cast<const struct params::metrics_params*>(
          repo.get_params("metrics"))->depth1_fname));
  m_depth1_collector->reset();
  m_task_collector.reset(new metrics::collectors::task_collector(
      static_cast<const struct params::metrics_params*>(
          repo.get_params("metrics"))->task_fname));
  m_task_collector->reset();

  ER_NOM("depth1_foraging loop functions initialization finished");
}

template<typename T>
bool foraging_loop_functions::init_cache_usage_penalty(
    argos::CFootBotEntity& robot) {

  T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

  if (controller.cache_acquired() && !controller.is_transporting_to_cache()) {
    ER_ASSERT(!controller.block_detected(), "FATAL: Block detected in cache?");

    /* Check whether the foot-bot is actually on a cache */
    int cache = robot_on_cache(robot);
    if (-1 != cache) {
      m_penalty_list.push_back(new cache_usage_penalty(&controller,
                                                       cache,
                                                       mc_cache_penalty,
                                                       GetSpace().GetSimulationClock()));
      return true;
    }
  }
  return false;
} /* init_cache_usage_penalty() */

template<typename T>
bool foraging_loop_functions::cache_usage_penalty_satisfied(
    argos::CFootBotEntity& robot) {

  T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

  auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                         [&](const cache_usage_penalty* p) {
                           return p->controller() == &controller;});
  if (it != m_penalty_list.end()) {
    return (*it)->penalty_satisfied(GetSpace().GetSimulationClock());
  }

  return false;
} /* cache_usage_penalty_satisfied() */

template<typename T>
void foraging_loop_functions::finish_cached_block_pickup(
    argos::CFootBotEntity& robot) {

  T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
  cache_usage_penalty* p = m_penalty_list.front();
  ER_ASSERT(p->controller() == &controller,
            "FATAL: Out of order cache penalty handling");

  events::cached_block_pickup pickup_op(rcppsw::er::g_server,
                                        &map()->caches()[p->cache_id()],
                                        robot_id(robot));
  m_penalty_list.remove(p);

  /*
   * Map must be called before controller for proper cache block decrement!
   */
  map()->accept(pickup_op);

  controller.visitor::template visitable_any<T>::accept(pickup_op);
  floor()->SetChanged();
} /* finish_cached_block_pickup() */

template<typename T>
bool foraging_loop_functions::handle_cache_block_drop(
    argos::CFootBotEntity& robot) {
  T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

  if (controller.cache_acquired() && controller.is_transporting_to_cache()) {
    /* Check whether the foot-bot is actually on a cache */
    int cache = robot_on_cache(robot);
    if (-1 != cache) {
      /* Update arena map state due to a block nest drop */
      events::cache_block_drop drop_op(rcppsw::er::g_server,
                                       controller.block(),
                                       &map()->caches()[cache]);

    map()->accept(drop_op);
    controller.visitor::template visitable_any<T>::accept(drop_op);
    return true;
    }
  }
  return false;
} /* handle_cache_block_drop() */

int foraging_loop_functions::robot_on_cache(const argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  return map()->robot_on_cache(pos);
} /* robot_on_cache() */

void foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
    controller::depth1::foraging_controller& controller =
        dynamic_cast<controller::depth1::foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /* get stats from this robot before its state changes */
    random_collector()->collect(controller);
    distance_collector()->collect(controller);
    depth0_collector()->collect(controller);
    m_depth1_collector->collect(controller);
    m_task_collector->collect(controller);

    /* Send the robot its new line of sight */
    set_robot_pos<controller::depth1::foraging_controller>(robot);
    set_robot_los<controller::depth1::foraging_controller>(robot);
    set_robot_tick<controller::depth1::foraging_controller>(robot);

    if (controller.is_carrying_block()) {
      handle_nest_block_drop<controller::depth1::foraging_controller>(robot);
      handle_cache_block_drop<controller::depth1::foraging_controller>(robot);
    } else { /* The foot-bot has no block item */
      handle_free_block_pickup<controller::depth1::foraging_controller>(robot);

      if (robot_serving_cache_penalty<controller::depth1::foraging_controller>(robot)) {
        if (cache_usage_penalty_satisfied<controller::depth1::foraging_controller>(robot)) {
          finish_cached_block_pickup<controller::depth1::foraging_controller>(robot);
        }
      } else {
        init_cache_usage_penalty<controller::depth1::foraging_controller>(robot);
      }
    }
} /* pre_step_iter() */

template<typename T>
bool foraging_loop_functions::robot_serving_cache_penalty(
    argos::CFootBotEntity& robot) {
  T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
  auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                         [&](const cache_usage_penalty* p) {
                           return p->controller() == &controller;});
  return it != m_penalty_list.end();
} /* robot_serving_cache_penalty() */

argos::CColor foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {

  /* The nest is a light gray */
  if (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (size_t i = 0; i < map()->caches().size(); ++i) {
    if (map()->caches()[i].contains_point(plane_pos)) {
      return argos::CColor::GRAY40;
    }
  } /* for(i..) */

  for (size_t i = 0; i < map()->blocks().size(); ++i) {
    if (map()->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::PreStep() {
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    pre_step_iter(robot);
  } /* for(it..) */
  pre_step_final();
} /* PreStep() */

void foraging_loop_functions::Reset() {
  depth0::foraging_loop_functions::Reset();
  m_depth1_collector->reset();
  m_task_collector->reset();
}

void foraging_loop_functions::Destroy() {
  depth0::foraging_loop_functions::Destroy();
  m_depth1_collector->finalize();
  m_task_collector->finalize();
}

void foraging_loop_functions::pre_step_final(void) {
  depth0::foraging_loop_functions::pre_step_final();

  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  if (map()->has_static_cache() && 0 == map()->caches().size()) {
    int n_foragers = m_task_collector->n_foragers();
    int n_collectors = m_task_collector->n_collectors();
    expressions::cache_respawn_probability p(mc_cache_respawn_scale_factor);
    if (p.calc(n_foragers, n_collectors) >=
        static_cast<double>(rand()) / RAND_MAX) {
      map()->static_cache_create();
    }
  }

  m_depth1_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_depth1_collector->reset_on_timestep();
  m_task_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_task_collector->reset_on_timestep();
} /* pre_step_final() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
typedef foraging_loop_functions depth1_foraging_loop_functions;
REGISTER_LOOP_FUNCTIONS(depth1_foraging_loop_functions, "depth1_foraging_loop_functions");

NS_END(depth1, support, fordyca);
