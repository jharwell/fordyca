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
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "fordyca/support/foraging_loop_functions.hpp"
#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/params/block_param_parser.hpp"
#include "fordyca/params/logging_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void) :
    m_arena_x(-4.7, 4.7),
    m_arena_y(-1.7, 1.7),
    m_nest_x(-3.5, -2.5),
    m_nest_y(-0.5, 0.5),
    m_floor(NULL),
    m_collector(),
    m_logging_params(),
    m_block_params(),
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
  m_param_manager.parse_all(node);

  m_logging_params.reset(static_cast<const struct logging_params*>(
      m_param_manager.get_params("logging")));

  m_param_manager.logging_init(std::make_shared<rcppsw::common::er_server>());
  m_floor = &GetSpace().GetFloorEntity();

  /* distribute blocks in arena */
  m_block_params.reset(static_cast<const struct block_params*>(
      m_param_manager.get_params("blocks")));
  m_blocks = std::make_shared<std::vector<argos::CVector2>>(m_block_params->n_blocks);
  m_distributor.reset(new support::block_distributor(m_arena_x,
                                                     m_arena_y,
                                                     m_nest_x,
                                                     m_nest_y,
                                                     m_block_params,
                                                     m_blocks));

  m_distributor->distribute_blocks();

  /* initialize stat collecting */
  m_collector.reset(m_logging_params->sim_stats);
}


void foraging_loop_functions::Reset() {
  m_collector.reset(m_logging_params->sim_stats);
  m_distributor->distribute_blocks();
}

void foraging_loop_functions::Destroy() {
  m_collector.finalize();
}

argos::CColor foraging_loop_functions::GetFloorColor(const argos::CVector2& plane_pos) {
  if (m_nest_x.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      m_nest_y.WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_blocks->size(); ++i) {
    if ((plane_pos - m_blocks->at(i)).SquareLength() < m_block_params->square_radius) {
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
    argos::CFootBotEntity& cFootBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    controller::foraging_controller& controller = dynamic_cast<controller::foraging_controller&>(cFootBot.GetControllableEntity().GetController());

    /* collect all stats from this robot */
    m_collector.collect_from_robot(controller);
    /* Get the position of the foot-bot on the ground as a CVector2 */
    argos::CVector2 pos;
    pos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
             cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    if (controller.carrying_block()) {
      /* TODO: possibly change this to be autonomous, rather than just
       * informing the robot... */
      /* Check whether the foot-bot is in the nest */
      if (m_nest_x.WithinMinBoundIncludedMaxBoundIncluded(pos.GetX()) &&
         m_nest_y.WithinMinBoundIncludedMaxBoundIncluded(pos.GetY())) {
        controller.publish_event(controller::foraging_controller::ENTERED_NEST);

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
      if (!m_nest_x.WithinMinBoundIncludedMaxBoundIncluded(pos.GetX()) &&
          !m_nest_y.WithinMinBoundIncludedMaxBoundIncluded(pos.GetY())) {
        /* Check whether the foot-bot is on a block item */
        for (size_t i = 0; i < m_blocks->size(); ++i) {
          if ((pos - m_blocks->at(i)).SquareLength() < m_block_params->square_radius) {
            /* If so, we move that item out of sight */
              m_blocks->at(i).Set(100.0f, 100.f);
            controller.pickup_block(i);

            /* The floor texture must be updated */
            m_floor->SetChanged();
            controller.publish_event(controller::foraging_controller::BLOCK_FOUND);
            break;
          }
        } /* for(i..) */
      }
    }
    ++i;
  } /* for(it..) */
  m_collector.store(GetSpace().GetSimulationClock());
}
using namespace argos;
REGISTER_LOOP_FUNCTIONS(foraging_loop_functions, "foraging_loop_functions")

NS_END(support, fordyca);
