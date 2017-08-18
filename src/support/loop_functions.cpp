/**
 * @file loop_functions.cpp
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
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/support/loop_functions.hpp"
#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/params/loop_function_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
loop_functions::loop_functions(void) :
    m_floor(NULL),
    m_collector(),
    mc_logging_params(),
    mc_loop_params(),
    m_map() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void loop_functions::Init(argos::TConfigurationNode& node) {
  m_floor = &GetSpace().GetFloorEntity();

  /* parse all environment parameters */
  params::loop_function_repository param_repo;
  param_repo.parse_all(node);

  /* Capture parsed parameters in logfile */
  std::ofstream init_file("loop-functions-init.txt");
  param_repo.show_all(init_file);
  init_file.close();

  mc_loop_params.reset(static_cast<const struct loop_functions_params*>(
      param_repo.get_params("loop_functions")));
  const struct grid_params * grid_params = static_cast<const struct grid_params*>(
      param_repo.get_params("grid"));

  /* initialize arena map and distribute blocks */
  m_map.reset(new representation::arena_map(grid_params,
                                            mc_loop_params->nest_x,
                                            mc_loop_params->nest_y));
  m_map->distribute_blocks(true);

  /* initialize stat collecting */
  m_collector.reset(new stat_collector(static_cast<const struct logging_params*>(
      param_repo.get_params("logging"))->sim_stats));
}

void loop_functions::Reset() {
  m_collector->reset();
  m_map->distribute_blocks(true);
}

void loop_functions::Destroy() {
  m_collector->finalize();
}

argos::CColor loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (mc_loop_params->nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      mc_loop_params->nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_map->blocks().size(); ++i) {
    if (m_map->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */
  return argos::CColor::WHITE;
} /* GetFloorColor() */

void loop_functions::PreStep() {
  int i = 0;

  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    controller::foraging_controller& controller =
        dynamic_cast<controller::foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /* get stats from this robot before its state changes */
    m_collector->collect_from_robot(controller);

    if (controller.is_carrying_block()) {
      if (controller.in_nest()) {
        representation::block& block = m_map->blocks()[controller.block_idx()];
        /*
         * Get stats from carried block before it's dropped and its state
         * changes.
         */
        m_collector->collect_from_block(block);

        /*
         * Handle updating the arena map state when a block is dropped in the
         * nest (must be before the actual drop because the block index goes to
         * -1 after that).
         */
        m_map->event_block_nest_drop(block);

        /* Actually drop the block item */
        controller.drop_block_in_nest();

        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (!controller.in_nest() && controller.block_detected()) {

        /* Check whether the foot-bot is actually on a block */
        int block = robot_on_block(*argos::any_cast<argos::CFootBotEntity*>(
            it->second));
        if (-1 != block) {
          m_map->event_block_pickup(m_map->blocks()[block], i);
          controller.pickup_block(block);

          /* The floor texture must be updated */
          m_floor->SetChanged();
          controller.publish_event(controller::foraging_controller::BLOCK_FOUND);
        }
      }
    }
    ++i;
  } /* for(it..) */
  m_collector->store_foraging_stats(GetSpace().GetSimulationClock());

}

int loop_functions::robot_on_block(const argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  return m_map->robot_on_block(pos);
} /* robot_on_block() */

bool loop_functions::IsExperimentFinished(void) {
  /*
   * If we are not respawning blocks and all blocks have been collected, signal
   * the end of the experiment. If respawn is enabled, then the experiment will
   * run until I cancel it.
   */

  if (!m_map->respawn_enabled() &&
      m_collector->n_collected_blocks() == m_map->n_blocks()) {
    return true;
  }
  return false;
} /* IsExperimentFinished() */


using namespace argos;
REGISTER_LOOP_FUNCTIONS(loop_functions, "foraging_loop_functions")

NS_END(support, fordyca);
