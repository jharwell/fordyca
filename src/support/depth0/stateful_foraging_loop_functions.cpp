/**
 * @file stateful_foraging_loop_functions.cpp
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
#include "fordyca/support/depth0/stateful_foraging_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/depth0/arena_interactor.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "rcppsw/er/server.hpp"
#include "fordyca/support/depth0/stateful_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using interactor =
    arena_interactor<controller::depth0::stateful_foraging_controller>;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_foraging_loop_functions::Init(ticpp::Element& node) {
  stateless_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth0_foraging loop functions");
  params::loop_function_repository repo(server_ref());

  repo.parse_all(node);
  rcppsw::er::g_server->log_stream() << repo;

  /* initialize stat collecting */
  auto* p_output = repo.parse_results<const struct params::output_params>();
  m_metrics_agg = rcppsw::make_unique<stateful_metrics_aggregator>(
      rcppsw::er::g_server, &p_output->metrics, output_root());

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller =
        dynamic_cast<controller::depth0::stateful_foraging_controller&>(
            robot.GetControllableEntity().GetController());
    auto* l_params = repo.parse_results<struct params::visualization_params>();

    controller.display_los(l_params->robot_los);
    utils::set_robot_los<controller::depth0::stateful_foraging_controller>(
        robot, *arena_map());
  } /* for(entity..) */
  ER_NOM("stateful_foraging loop functions initialization finished");
}

void stateful_foraging_loop_functions::pre_step_iter(
    argos::CFootBotEntity& robot) {
  auto& controller =
      static_cast<controller::depth0::stateful_foraging_controller&>(
          robot.GetControllableEntity().GetController());

  /* collect metrics from robot before its state changes */
  m_metrics_agg->collect_from_controller(&controller);

  /* Send the robot its new line of sight */
  utils::set_robot_pos<controller::depth0::stateful_foraging_controller>(robot);
  utils::set_robot_los<controller::depth0::stateful_foraging_controller>(
      robot, *arena_map());
  set_robot_tick<controller::depth0::stateful_foraging_controller>(robot);

  /* Now watch it react to the environment */
  interactor(rcppsw::er::g_server, arena_map(), m_metrics_agg.get(), floor())(
      controller,
      GetSpace().GetSimulationClock());
} /* pre_step_iter() */

__rcsw_pure argos::CColor stateful_foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (arena_map()->nest().contains_point(plane_pos)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }

  for (size_t i = 0; i < arena_map()->blocks().size(); ++i) {
    if (arena_map()->blocks()[i]->contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void stateful_foraging_loop_functions::PreStep() {
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  pre_step_final();
} /* PreStep() */

void stateful_foraging_loop_functions::Reset(void) {
  stateless_foraging_loop_functions::Reset();
  m_metrics_agg->reset_all();
} /* Reset() */

void stateful_foraging_loop_functions::pre_step_final(void) {
  stateless_foraging_loop_functions::pre_step_final();
  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
  m_metrics_agg->timestep_inc_all();
} /* pre_step_final() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(stateful_foraging_loop_functions,
                        "stateful_foraging_loop_functions"); // NOLINT

NS_END(depth0, support, fordyca);
