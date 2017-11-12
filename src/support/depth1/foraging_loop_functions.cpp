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
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/diagnostics_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/loop_function_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  depth0::foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1_foraging loop functions");
  params::loop_function_repository repo;

  repo.parse_all(node);

  /* initialize stat collecting */
  m_robot_collector.reset(new diagnostics::depth1::collector(
      static_cast<const struct params::diagnostics_params*>(
          repo.get_params("logging"))->robot_fname));
  m_robot_collector->reset();

  ER_NOM("depth1_foraging loop functions initialization finished");
}

void foraging_loop_functions::handle_cached_block_pickup(
    argos::CFootBotEntity& robot) {

  controller::depth1::foraging_controller& controller =
      static_cast<controller::depth1::foraging_controller&>(
          robot.GetControllableEntity().GetController());

  if (controller.is_exploring_for_cache() && controller.cache_detected()) {
    ER_ASSERT(!controller.block_detected(), "FATAL: Block detected in cache?");

    /* Check whether the foot-bot is actually on a cache */
    int cache = robot_on_cache(robot);
    if (-1 != cache) {
      events::cached_block_pickup pickup_op(rcppsw::common::g_server,
                                          &map()->caches()[cache],
                                          robot_id(robot));
      controller.visitor::visitable_any<controller::depth1::foraging_controller>::accept(pickup_op);
      map()->accept(pickup_op);
    }
  }
} /* handle_cached_block_pickup() */

void foraging_loop_functions::handle_cache_block_drop(
    argos::CFootBotEntity& robot) {
  controller::depth1::foraging_controller& controller =
      static_cast<controller::depth1::foraging_controller&>(
          robot.GetControllableEntity().GetController());

  if (controller.cache_detected()) {
    /* get stats from this robot before its state changes */
    /* random_foraging_loop_functions::robot_collector()->collect(controller); */

    /* Check whether the foot-bot is actually on a cache */
    int cache = robot_on_cache(robot);
    if (-1 != cache) {
      /* Update arena map state due to a block nest drop */
      events::cache_block_drop drop_op(rcppsw::common::g_server,
                                       controller.block(),
                                       &map()->caches()[cache]);

    /* TODO: Get stats from carried block before it's dropped */
    /* block_collector()->accept(drop_op); */

    map()->accept(drop_op);

    /* Actually drop the block */
    controller.visitor::visitable_any<controller::depth1::foraging_controller>::accept(drop_op);
    }
  }
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
    m_robot_collector->collect(controller);

    /* Send the robot its new line of sight */
    depth0::foraging_loop_functions::set_robot_los(robot);
    depth0::foraging_loop_functions::set_robot_tick(robot);

    if (controller.is_carrying_block()) {
      depth0::foraging_loop_functions::handle_nest_block_drop(controller);
      handle_cache_block_drop(robot);
    } else { /* The foot-bot has no block item */
      depth0::foraging_loop_functions::handle_free_block_pickup(robot);
      handle_cached_block_pickup(robot);
    }
} /* pre_step_iter() */

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
  depth0::foraging_loop_functions::pre_step_final();
} /* PreStep() */

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
