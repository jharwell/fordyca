/**
 * @file stateless_loop_functions.cpp
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
#include "fordyca/support/depth0/stateless_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>

#include "fordyca/controller/depth0/stateless_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_loop_functions::stateless_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.stateless"),
      m_arena_map(nullptr),
      m_interactor(nullptr) {}

stateless_loop_functions::~stateless_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_loop_functions::Init(ticpp::Element& node) {
  base_loop_functions::Init(node);
  ndc_push();
  ER_INFO("Initializing...");

  /* initialize output and metrics collection */
  auto* arena = params().parse_results<params::arena::arena_map_params>();
  params::output_params output =
      *params().parse_results<params::output_params>();
  output.metrics.arena_grid = arena->grid;

  m_metrics_agg =
      rcppsw::make_unique<stateless_metrics_aggregator>(&output.metrics,
                                                        output_root());

  /* initialize arena map and distribute blocks */
  arena_map_init(params());

  auto* arenap = params().parse_results<params::arena::arena_map_params>();
  m_interactor =
      rcppsw::make_unique<interactor>(arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      &arenap->blocks.manipulation_penalty);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = static_cast<controller::base_controller&>(
        robot.GetControllableEntity().GetController());

    /*
     * If NULL, then visualization has been disabled.
     */
    auto* vparams =
        params().parse_results<struct params::visualization_params>();
    if (nullptr != vparams) {
      controller.display_id(vparams->robot_id);
    }
  } /* for(&robot..) */
  ER_INFO("Initialization finished");
  ndc_pop();
}

void stateless_loop_functions::Reset() {
  m_metrics_agg->reset_all();
  m_arena_map->distribute_all_blocks();
}

void stateless_loop_functions::Destroy() { m_metrics_agg->finalize_all(); }

__rcsw_pure argos::CColor stateless_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (m_arena_map->nest().contains_point(plane_pos)) {
    return argos::CColor(m_arena_map->nest().color().red(),
                         m_arena_map->nest().color().green(),
                         m_arena_map->nest().color().blue());
  }

  for (auto& block : m_arena_map->blocks()) {
    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (block->contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void stateless_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto& controller = static_cast<controller::depth0::stateless_controller&>(
      robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_metrics_agg->collect_from_controller(&controller);
  controller.free_pickup_event(false);
  controller.free_drop_event(false);

  /* Send the robot its current position */
  set_robot_tick<controller::depth0::stateless_controller>(robot);
  loop_utils::set_robot_pos<controller::depth0::stateless_controller>(robot);

  /* update arena map metrics with robot position */
  auto coord = math::rcoord_to_dcoord(controller.robot_loc(),
                                      m_arena_map->grid_resolution());
  m_arena_map->access<arena_grid::kRobotOccupancy>(coord) = true;

  /* Now watch it react to the environment */
  (*m_interactor)(controller, GetSpace().GetSimulationClock());
} /* pre_step_iter() */

void stateless_loop_functions::pre_step_final(void) {
  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

void stateless_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  m_metrics_agg->collect_from_arena(m_arena_map.get());
  m_metrics_agg->collect_from_loop(this);
  pre_step_final();
  ndc_pop();
} /* PreStep() */

void stateless_loop_functions::arena_map_init(
    params::loop_function_repository& repo) {
  auto* aparams = repo.parse_results<struct params::arena::arena_map_params>();
  auto* vparams = repo.parse_results<struct params::visualization_params>();

  m_arena_map.reset(new ds::arena_map(aparams));
  if (!m_arena_map->initialize()) {
    ER_ERR("Could not initialize arena map");
    std::exit(EXIT_FAILURE);
  }
  m_arena_map->distribute_all_blocks();

  /*
   * If null, visualization has been disabled.
   */
  if (nullptr != vparams) {
    for (auto& block : m_arena_map->blocks()) {
      block->display_id(vparams->block_id);
    } /* for(&block..) */
  }
} /* arena_map_init() */

using namespace argos;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_LOOP_FUNCTIONS(stateless_loop_functions,
                        "stateless_loop_functions"); // NOLINT
#pragma clang diagnostic pop
NS_END(depth0, support, fordyca);
