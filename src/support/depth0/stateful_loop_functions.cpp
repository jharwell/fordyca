/**
 * @file stateful_loop_functions.cpp
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
#include "fordyca/support/depth0/stateful_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/controller/depth0/stateful_controller.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/depth0/stateful_metrics_aggregator.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_loop_functions::stateful_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.stateful"),
      m_metrics_agg(nullptr) {}

stateful_loop_functions::~stateful_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_loop_functions::Init(ticpp::Element& node) {
  stateless_loop_functions::Init(node);
  ndc_push();
  ER_INFO("Initializing...");

  /* initialize stat collecting */
  auto* arenap = params().parse_results<params::arena::arena_map_params>();
  params::output_params output =
      *params().parse_results<const struct params::output_params>();
  output.metrics.arena_grid = arenap->grid;

  m_metrics_agg =
      rcppsw::make_unique<stateful_metrics_aggregator>(&output.metrics,
                                                       output_root());

  /* intitialize robot interactions with environment */
  m_interactor =
      rcppsw::make_unique<interactor>(arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      &arenap->blocks.manipulation_penalty);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller =
        dynamic_cast<controller::depth0::stateful_controller&>(
            robot.GetControllableEntity().GetController());

    /*
     * If NULL, then visualization has been disabled.
     */
    auto* vparams = params().parse_results<struct params::visualization_params>();
    if (nullptr != vparams) {
      controller.display_los(vparams->robot_los);
    }
  } /* for(entity..) */
  ER_INFO("Initialization finished");
  ndc_pop();
}

void stateful_loop_functions::pre_step_iter(
    argos::CFootBotEntity& robot) {
  auto& controller =
      static_cast<controller::depth0::stateful_controller&>(
          robot.GetControllableEntity().GetController());

  /* collect metrics from robot before its state changes */
  m_metrics_agg->collect_from_controller(&controller);
  controller.free_pickup_event(false);
  controller.free_drop_event(false);

  /* Send the robot its new line of sight */
  utils::set_robot_pos<decltype(controller)>(robot);
  utils::set_robot_los<decltype(controller)>(robot, *arena_map());
  set_robot_tick<decltype(controller)>(robot);

  /* update arena map metrics with robot position */
  auto coord = math::rcoord_to_dcoord(controller.robot_loc(),
                                      arena_map()->grid_resolution());
  arena_map()->access<arena_grid::kRobotOccupancy>(coord) = true;

  /* Now watch it react to the environment */
  (*m_interactor)(controller, GetSpace().GetSimulationClock());
} /* pre_step_iter() */

__rcsw_pure argos::CColor stateful_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (arena_map()->nest().contains_point(plane_pos)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }

  for (size_t i = 0; i < arena_map()->blocks().size(); ++i) {
    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (arena_map()->blocks()[i]->contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void stateful_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  m_metrics_agg->collect_from_arena(arena_map());
  m_metrics_agg->collect_from_loop(this);
  pre_step_final();
  ndc_pop();
} /* PreStep() */

void stateful_loop_functions::Reset(void) {
  stateless_loop_functions::Reset();
  m_metrics_agg->reset_all();
} /* Reset() */

void stateful_loop_functions::pre_step_final(void) {
  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_LOOP_FUNCTIONS(stateful_loop_functions,
                        "stateful_loop_functions");
#pragma clang diagnostic pop;
NS_END(depth0, support, fordyca);
