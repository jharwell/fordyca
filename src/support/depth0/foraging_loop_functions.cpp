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
#include "fordyca/support/depth0/foraging_loop_functions.hpp"
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/depth0/foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/params/loop_functions_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

namespace robot_collectors = metrics::collectors::robot_metrics;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  random_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth0_foraging loop functions");
  params::loop_function_repository repo;

  repo.parse_all(node);

  /* initialize stat collecting */
  m_collector.reset(new robot_collectors::depth0_collector(
      static_cast<const struct params::metrics_params*>(
          repo.get_params("metrics"))->depth0_fname));
  m_collector->reset();

  /* configure robots */
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    controller::base_foraging_controller& controller =
        dynamic_cast<controller::base_foraging_controller&>(
            robot.GetControllableEntity().GetController());
    const struct params::loop_functions_params * l_params =
        static_cast<const struct params::loop_functions_params*>(
            repo.get_params("loop_functions"));

    controller.display_los(l_params->display_robot_los);
    set_robot_los<controller::depth0::foraging_controller>(robot);
  } /* for(it..) */
  ER_NOM("depth0_foraging loop functions initialization finished");
}

void foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
    controller::depth0::foraging_controller& controller =
        static_cast<controller::depth0::foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /* get stats from this robot before its state changes */
    distance_collector()->collect(controller);
    random_collector()->collect(controller);
    m_collector->collect(controller);

    /* Send the robot its new line of sight */
    set_robot_pos<controller::depth0::foraging_controller>(robot);
    set_robot_los<controller::depth0::foraging_controller>(robot);
    set_robot_tick<controller::depth0::foraging_controller>(robot);

    if (controller.is_carrying_block()) {
      handle_nest_block_drop<controller::depth0::foraging_controller>(robot);
    } else { /* The foot-bot has no block item */
      handle_free_block_pickup<controller::depth0::foraging_controller>(robot);
    }
} /* pre_step_iter() */

template<typename T>
bool foraging_loop_functions::handle_free_block_pickup(argos::CFootBotEntity& robot) {

 T&  controller = static_cast<T&>(robot.GetControllableEntity().GetController());

  if (!controller.in_nest() && controller.is_acquiring_block() &&
      controller.block_detected()) {
    /* Check whether the foot-bot is actually on a block */
    int block = robot_on_block(robot);
    if (-1 != block) {
      events::free_block_pickup pickup_op(rcppsw::er::g_server,
                                          &map()->blocks()[block],
                                          robot_id(robot));
      controller.visitor::template visitable_any<T>::accept(pickup_op);
      map()->accept(pickup_op);

      /* The floor texture must be updated */
      floor()->SetChanged();
      return true;
    }
  }
  return false;
} /* handle_free_block_pickup() */

template <typename T>
bool foraging_loop_functions::handle_nest_block_drop(argos::CFootBotEntity& robot) {
  T&  controller = static_cast<T&>(robot.GetControllableEntity().GetController());
  if (controller.in_nest() && controller.is_transporting_to_nest()) {

    /* Update arena map state due to a block nest drop */
    events::nest_block_drop drop_op(rcppsw::er::g_server,
                                    controller.block());

    /* Get stats from carried block before it's dropped */
    random_foraging_loop_functions::block_collector()->accept(drop_op);

    map()->accept(drop_op);

    /* Actually drop the block */
    controller.visitor::template visitable_any<T>::accept(drop_op);

    /* The floor texture must be updated */
    floor()->SetChanged();
    return true;
  }
  return false;
} /* handle_nest_block_drop() */

argos::CColor foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {

  /* The nest is a light gray */
  if (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  for (size_t i = 0; i < map()->blocks().size(); ++i) {
    if (map()->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::Destroy(void) {
  random_foraging_loop_functions::Destroy();
  m_collector->finalize();
}

void foraging_loop_functions::Reset(void) {
  random_foraging_loop_functions::Reset();
  m_collector->reset();
}

void foraging_loop_functions::pre_step_final(void) {
  random_foraging_loop_functions::pre_step_final();
  m_collector->reset_on_timestep();
} /* pre_step_final() */

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

template<typename T>
void foraging_loop_functions::set_robot_los(argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  representation::discrete_coord robot_loc =
      representation::real_to_discrete_coord(pos, map()->grid_resolution());
  T& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  std::unique_ptr<representation::line_of_sight> new_los =
      rcppsw::make_unique<representation::line_of_sight>(
          map()->subgrid(robot_loc.first, robot_loc.second, 1),
          robot_loc);
  controller.los(new_los);
} /* set_robot_los() */

template<typename T>
void foraging_loop_functions::set_robot_tick(argos::CFootBotEntity& robot) {
  T& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  controller.tick(GetSpace().GetSimulationClock() + 1); /* for next timestep */
} /* set_robot_tic() */

robot_collectors::depth0_collector* foraging_loop_functions::depth0_collector(void) const {
  return m_collector.get();
} /* depth0_collector() */


template void foraging_loop_functions::set_robot_los<controller::depth1::foraging_controller>(argos::CFootBotEntity&);
template void foraging_loop_functions::set_robot_tick<controller::depth1::foraging_controller>(argos::CFootBotEntity&);
template bool foraging_loop_functions::handle_nest_block_drop<controller::depth1::foraging_controller>(argos::CFootBotEntity&);
template bool foraging_loop_functions::handle_free_block_pickup<controller::depth1::foraging_controller>(argos::CFootBotEntity&);

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
typedef foraging_loop_functions depth0_foraging_loop_functions;
REGISTER_LOOP_FUNCTIONS(depth0_foraging_loop_functions, "depth0_foraging_loop_functions");

NS_END(depth0, support, fordyca);
