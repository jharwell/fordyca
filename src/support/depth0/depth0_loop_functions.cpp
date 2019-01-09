/**
 * @file depth0_loop_functions.cpp
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
#include "fordyca/support/depth0/depth0_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"
#include "rcppsw/swarm/convergence/convergence_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using ds::arena_grid;
namespace rswc = rcppsw::swarm::convergence;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth0_loop_functions::depth0_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth0"), m_metrics_agg(nullptr) {}

depth0_loop_functions::~depth0_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth0_loop_functions::Init(ticpp::Element& node) {
  base_loop_functions::Init(node);
  ndc_push();
  ER_INFO("Initializing...");

  /* initialize output and metrics collection */
  auto* arena = params()->parse_results<params::arena::arena_map_params>();
  params::output_params output =
      *params()->parse_results<params::output_params>();
  auto* conv = params()->parse_results<rswc::convergence_params>();
  output.metrics.arena_grid = arena->grid;

  m_metrics_agg = rcppsw::make_unique<depth0_metrics_aggregator>(
      &output.metrics, conv, output_root());

  /* intitialize robot interactions with environment */
  auto* arenap = params()->parse_results<params::arena::arena_map_params>();
  m_crw_interactor =
      rcppsw::make_unique<crw_itype>(arena_map(),
                                     m_metrics_agg.get(),
                                     floor(),
                                     &arenap->blocks.manipulation_penalty);
  m_dpo_interactor =
      rcppsw::make_unique<dpo_itype>(arena_map(),
                                     m_metrics_agg.get(),
                                     floor(),
                                     &arenap->blocks.manipulation_penalty);
  m_mdpo_interactor =
      rcppsw::make_unique<mdpo_itype>(arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      &arenap->blocks.manipulation_penalty);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::base_controller&>(
        robot.GetControllableEntity().GetController());
    controller_configure(&controller);
  } /* for(entity..) */
  ER_INFO("Initialization finished");
  ndc_pop();
}

void depth0_loop_functions::controller_configure(
    controller::base_controller* const c) {
  auto* mdpo = dynamic_cast<controller::depth0::mdpo_controller*>(c);
  /*
   * If NULL, then visualization has been disabled.
   */
  auto* vparams = params()->parse_results<struct params::visualization_params>();
  if (nullptr != mdpo && nullptr != vparams) {
    mdpo->display_los(vparams->robot_los);
  }
  if (nullptr != vparams) {
    c->display_id(vparams->robot_id);
  }
} /* controller_configure() */

void depth0_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto& controller = static_cast<controller::depth0::depth0_controller&>(
      robot.GetControllableEntity().GetController());

  /*
   * This is the one place in the depth0 event handling code where we need to
   * know *ALL* of the controllers that are in the depth, and we can't/don't use
   * templates and/or inheritance to get what we need.
   */
  auto* dpo = dynamic_cast<controller::depth0::dpo_controller*>(&controller);
  auto* mdpo = dynamic_cast<controller::depth0::mdpo_controller*>(&controller);
  auto* crw = dynamic_cast<controller::depth0::crw_controller*>(&controller);

  /* collect metrics from robot before its state changes */
  if (nullptr != mdpo) {
    m_metrics_agg->collect_from_controller(mdpo);
  } else if (nullptr != dpo) {
    m_metrics_agg->collect_from_controller(dpo);
  } else if (nullptr != crw) {
    m_metrics_agg->collect_from_controller(crw);
  }

  controller.free_pickup_event(false);
  controller.free_drop_event(false);

  /* Send the robot its new line of sight */
  loop_utils::set_robot_pos<decltype(controller)>(robot);
  set_robot_tick<decltype(controller)>(robot);

  if (nullptr != dpo || nullptr != mdpo) {
    ER_ASSERT(std::fmod(dpo->los_dim(), arena_map()->grid_resolution()) <=
                  std::numeric_limits<double>::epsilon(),
              "LOS dimension (%f) not an even multiple of grid resolution (%f)",
              dpo->los_dim(),
              arena_map()->grid_resolution());
    uint los_grid_size = dpo->los_dim() / arena_map()->grid_resolution();
    loop_utils::set_robot_los<decltype(*dpo)>(robot, los_grid_size, *arena_map());
  }

  /*
   * Now update the robot's state as a result of arena interactions. For MDPO
   * controllers, the DPO pointer will *ALSO* be non-null, so the order of the
   * if()s here is important.
   */
  if (nullptr != mdpo) {
    (*m_mdpo_interactor)(*mdpo, GetSpace().GetSimulationClock());
  } else if (nullptr != dpo) {
    (*m_dpo_interactor)(*dpo, GetSpace().GetSimulationClock());
  } else if (nullptr != crw) {
    (*m_crw_interactor)(*crw, GetSpace().GetSimulationClock());
  } else {
    ER_FATAL_SENTINEL("Bad depth0 controller");
  }

  auto coord =
      rmath::dvec2uvec(controller.position(), arena_map()->grid_resolution());
  arena_map()->access<arena_grid::kRobotOccupancy>(coord) = true;
} /* pre_step_iter() */

__rcsw_pure argos::CColor depth0_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
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
    if (arena_map()->blocks()[i]->contains_point(tmp)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth0_loop_functions::PreStep(void) {
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

void depth0_loop_functions::Destroy(void) { m_metrics_agg->finalize_all(); }
void depth0_loop_functions::Reset(void) {
  ndc_push();
  base_loop_functions::Reset();
  m_metrics_agg->reset_all();
  ndc_pop();
} /* Reset() */

void depth0_loop_functions::pre_step_final(void) {
  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

/*******************************************************************************
 * Temporal Variance Metrics
 ******************************************************************************/
double depth0_loop_functions::env_block_manipulation(void) const {
  return m_crw_interactor->block_manip_penalty(
      const_cast<depth0_loop_functions*>(this)->GetSpace().GetSimulationClock());
} /* env_block_manipulation() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_LOOP_FUNCTIONS(depth0_loop_functions, "depth0_loop_functions");
#pragma clang diagnostic pop;
NS_END(depth0, support, fordyca);
