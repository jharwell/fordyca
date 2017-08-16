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
    mc_block_params(),
    mc_loop_params(),
    m_distributor(),
    m_blocks() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void loop_functions::Init(argos::TConfigurationNode& node) {
  params::loop_function_repository param_repo;

  /* parse all environment parameters */
  param_repo.parse_all(node);

  mc_loop_params.reset(static_cast<const struct loop_functions_params*>(
      param_repo.get_params("loop_functions")));
  mc_logging_params.reset(static_cast<const struct logging_params*>(
      param_repo.get_params("logging")));

  std::ofstream init_file("loop-functions-init.txt");
  param_repo.show_all(init_file);
  init_file.close();

  m_floor = &GetSpace().GetFloorEntity();

  /* distribute blocks in arena */
  mc_block_params.reset(static_cast<const struct block_params*>(
      param_repo.get_params("blocks")));
  m_blocks = std::make_shared<std::vector<representation::block>>(
      mc_block_params->n_blocks,
      representation::block(mc_block_params->dimension));
  m_distributor.reset(new support::block_distributor(mc_loop_params->arena_x,
                                                     mc_loop_params->arena_y,
                                                     mc_loop_params->nest_x,
                                                     mc_loop_params->nest_y,
                                                     mc_block_params,
                                                     m_blocks));

  m_distributor->distribute_blocks(true);

  /* initialize stat collecting */
  m_collector.reset(mc_logging_params->sim_stats);
}

void loop_functions::Reset() {
  m_collector.reset(mc_logging_params->sim_stats);
  m_distributor->distribute_blocks();
}

void loop_functions::Destroy() {
  m_collector.finalize();
}

argos::CColor loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (mc_loop_params->nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      mc_loop_params->nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_blocks->size(); ++i) {
    if (m_blocks->at(i).contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  }
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
    controller::controller& controller =
        dynamic_cast<controller::controller&>(
        robot.GetControllableEntity().GetController());

    /* get stats from this robot before its state changes */
    m_collector.collect_from_robot(controller);

    if (controller.is_carrying_block()) {
      if (controller.in_nest()) {
        /*
         * Get stats from carried block before it's dropped and its state
         * changes.
         */
        m_collector.collect_from_block(m_blocks->at(controller.block_idx()));

        m_blocks->at(controller.block_idx()).update_on_nest_drop();
        /*
         * Place a new block item on the ground (must be before the actual drop
         * because the block index goes to -1 after that).
         */
        m_distributor->distribute_block(controller.block_idx());

        /* Actually drop the block item */
        controller.drop_block_in_nest();
        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (!controller.in_nest() && controller.block_detected()) {

        /* Check whether the foot-bot is on a block item */
        int block = robot_on_block(*argos::any_cast<argos::CFootBotEntity*>(
            it->second));
        if (-1 != block) {
          m_blocks->at(block).update_on_robot_pickup(i);
          controller.pickup_block(block);

          /* The floor texture must be updated */
          m_floor->SetChanged();
          controller.publish_event(controller::controller::BLOCK_FOUND);
        }
      }
    }
    ++i;
  } /* for(it..) */
  m_collector.store_stats(GetSpace().GetSimulationClock());

}

int loop_functions::robot_on_block(const argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  for (size_t i = 0; i < m_blocks->size(); ++i) {
    if (m_blocks->at(i).contains_point(pos)) {
        return i;
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

bool loop_functions::IsExperimentFinished(void) {
  /*
   * If we are not respawning blocks and all blocks have been collected, signal
   * the end of the experiment. If respawn is enabled, then the experiment will
   * run until I cancel it.
   */
  if (!mc_block_params->respawn &&
      m_collector.n_collected_blocks() == mc_block_params->n_blocks) {
    return true;
  }
  return false;
} /* IsExperimentFinished() */


using namespace argos;
REGISTER_LOOP_FUNCTIONS(loop_functions, "loop_functions")

NS_END(support, fordyca);
