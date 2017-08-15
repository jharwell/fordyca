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
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/support/foraging_loop_functions.hpp"
#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/params/block_param_parser.hpp"
#include "fordyca/params/logging_param_parser.hpp"
#include "fordyca/params/loop_functions_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void) :
    m_floor(NULL),
    m_collector(),
    mc_logging_params(),
    mc_block_params(),
    mc_loop_params(),
    m_param_manager(),
    m_distributor(),
    m_blocks() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  /* parse all environment parameters */
  m_param_manager.add_category("blocks", new params::block_param_parser());
  m_param_manager.add_category("logging", new params::logging_param_parser());
  m_param_manager.add_category("loop_functions",
                               new params::loop_functions_param_parser());
  m_param_manager.parse_all(node);

  mc_loop_params.reset(static_cast<const struct loop_functions_params*>(
      m_param_manager.get_params("loop_functions")));
  mc_logging_params.reset(static_cast<const struct logging_params*>(
      m_param_manager.get_params("logging")));

  m_param_manager.logging_init(std::make_shared<rcppsw::common::er_server>("loop-functions-init.txt"));
  m_param_manager.show_all();
  m_floor = &GetSpace().GetFloorEntity();

  /* distribute blocks in arena */
  mc_block_params.reset(static_cast<const struct block_params*>(
      m_param_manager.get_params("blocks")));
  m_blocks = std::make_shared<std::vector<argos::CVector2>>(mc_block_params->n_blocks);
  m_distributor.reset(new support::block_distributor(mc_loop_params->arena_x,
                                                     mc_loop_params->arena_y,
                                                     mc_loop_params->nest_x,
                                                     mc_loop_params->nest_y,
                                                     mc_block_params,
                                                     m_blocks));

  m_distributor->distribute_blocks();

  /* initialize stat collecting */
  m_collector.reset(mc_logging_params->sim_stats);
}


void foraging_loop_functions::Reset() {
  m_collector.reset(mc_logging_params->sim_stats);
  m_distributor->distribute_blocks();
}

void foraging_loop_functions::Destroy() {
  m_collector.finalize();
}

argos::CColor foraging_loop_functions::GetFloorColor(const argos::CVector2& plane_pos) {
  if (mc_loop_params->nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      mc_loop_params->nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_blocks->size(); ++i) {
    if (point_within_block(plane_pos, i)) {
      return argos::CColor::BLACK;
    }
  }
  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::PreStep() {
  int i = 0;

  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    controller::foraging_controller& controller = dynamic_cast<controller::foraging_controller&>(robot.GetControllableEntity().GetController());
    /* collect all stats from this robot */
    m_collector.collect_from_robot(controller);
    /* Get the position of the foot-bot on the ground as a CVector2 */
    if (controller.is_carrying_block()) {
      /* Check whether the foot-bot is in the nest */
      if (controller.in_nest()) {
        /*
         * Place a new block item on the ground (must be before the actual drop
         * because the block index goes to -1 after that).
         */
        m_distributor->distribute_block(controller.block_idx());
        /* Drop the block item */
        controller.drop_block_in_nest();

        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (!controller.in_nest() && controller.block_detected()) {

        /* Check whether the foot-bot is on a block item */
        int block = robot_on_block(*argos::any_cast<argos::CFootBotEntity*>(it->second));
        if (-1 == block) {
          printf("FALSE positive on robot%d_on_block\n", i);
        } else {
          /* Move that item out of sight */
          m_blocks->at(block).Set(100.0f, 100.f);
          controller.pickup_block(block);

          /* The floor texture must be updated */
          m_floor->SetChanged();
          controller.publish_event(controller::foraging_controller::BLOCK_FOUND);
        }
      }
    }
    ++i;
  } /* for(it..) */
  m_collector.store(GetSpace().GetSimulationClock());
}

int foraging_loop_functions::robot_on_block(const argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  for (size_t i = 0; i < m_blocks->size(); ++i) {
    if (point_within_block(pos, i)) {
        return i;
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

bool foraging_loop_functions::point_within_block(const argos::CVector2& point,
                                                 size_t index) {
  double x = m_blocks->at(index).GetX();
  double y = m_blocks->at(index).GetY();
  if (point.GetX() < (x + (.5 * mc_block_params->dimension)) &&
      point.GetX() > (x - (.5 * mc_block_params->dimension)) &&
      point.GetY() < (y + (.5 * mc_block_params->dimension)) &&
      point.GetY() > (y - (.5 * mc_block_params->dimension))) {
    return true;
  }
  return false;
} /* point_within_block() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(foraging_loop_functions, "foraging_loop_functions")

NS_END(support, fordyca);
