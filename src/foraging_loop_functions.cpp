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
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "fordyca/foraging_loop_functions.hpp"
#include "fordyca/foraging_controller.hpp"
#include "fordyca/block_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void) :
    m_arena_x(-0.9f, 1.7f),
    m_arena_y(-1.7f, 1.7f),
    m_block_pos(),
    m_floor(NULL),
    m_rng(NULL),
    m_ofname(),
    m_ofile(),
    m_total_collected_blocks(0),
    m_block_params(),
    m_param_manager() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& t_node) {
  argos::TConfigurationNode& foraging = argos::GetNode(t_node, "foraging");
  m_param_manager.init(std::make_shared<rcppsw::common::er_server>("loop-functions.txt"));
  m_param_manager.add_category("block", new block_param_parser());
  m_param_manager.parse_all(foraging);
  m_block_params.reset(static_cast<const struct block_params*>(m_param_manager.get_params("block")));
  m_floor = &GetSpace().GetFloorEntity();
  m_rng = argos::CRandom::CreateRNG("argos");

  /*
   * Distribute block items uniformly in the arena.
   */
  for (size_t i = 0; i < m_block_params->n_items; ++i) {
    m_block_pos.push_back(
        argos::CVector2(m_rng->Uniform(m_arena_x),
                        m_rng->Uniform(m_arena_y)));
  } /* for(i..) */

  /* Open output file and truncate */
  argos::GetNodeAttribute(foraging, "output", m_ofname);
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);

  m_ofile << "clock\tcollected_blocks\tresting\texploring\treturning\tcollision_avoidance\n";
}

void foraging_loop_functions::Reset() {
  m_total_collected_blocks = 0;
  m_ofile.close();
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_ofile << "clock\tcollected_blocks\tresting\texploring\treturning\tcollision_avoidance\n";

  for (size_t i = 0; i < m_block_pos.size(); ++i) {
    m_block_pos[i].Set(m_rng->Uniform(m_arena_x),
                      m_rng->Uniform(m_arena_y));
  } /* for(i..) */
}

void foraging_loop_functions::Destroy() {
  m_ofile.close();
}

argos::CColor foraging_loop_functions::GetFloorColor(const argos::CVector2& plane_pos) {
  if (plane_pos.GetX() < -1.0f) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_block_pos.size(); ++i) {
    if ((plane_pos - m_block_pos[i]).SquareLength() < m_block_params->square_radius) {
      return argos::CColor::BLACK;
    }
  }
  return argos::CColor::WHITE;
} /* get_floor_color() */

void foraging_loop_functions::PreStep() {
  /*
   * If a robot is in the nest, drop the block item
   * If a robot is on a block item, pick it
   * Each robot can carry only one block item per time
   */
  uint n_resting = 0;
  uint n_exploring = 0;
  uint n_returning = 0;
  uint n_avoiding = 0;

  int i = 0;

  /* Check whether a robot is on a block item */
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& cFootBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    foraging_controller& controller = dynamic_cast<foraging_controller&>(cFootBot.GetControllableEntity().GetController());

    /* Count how many foot-bots are in which state */
    n_resting += controller.is_resting();
    n_exploring += controller.is_exploring();
    n_returning += controller.is_returning();
    n_avoiding += controller.is_avoiding_collision();

    /* Get the position of the foot-bot on the ground as a CVector2 */
    argos::CVector2 pos;
    pos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
             cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    if (controller.carrying_block()) {
      /* TODO: possibly change this to be autonomous, rather than just
       * informing the robot... */
      /* Check whether the foot-bot is in the nest */
      if(pos.GetX() < -1.0f) {
        controller.publish_event(foraging_controller::ENTERED_NEST);

        /*
         * Place a new block item on the ground (must be before the actual drop
         * because the block index goes to -1 after that)
         */
        m_block_pos[controller.block_idx()].Set(m_rng->Uniform(m_arena_x),
                                               m_rng->Uniform(m_arena_y));
        /* Drop the block item */
        controller.drop_block_in_nest();
        ++m_total_collected_blocks;

        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (pos.GetX() > -1.0f) {
        /* Check whether the foot-bot is on a block item */
        for (size_t i = 0; i < m_block_pos.size(); ++i) {
          if((pos - m_block_pos[i]).SquareLength() < m_block_params->square_radius) {
            /* If so, we move that item out of sight */
            m_block_pos[i].Set(100.0f, 100.f);
            controller.pickup_block(i);

            /* The floor texture must be updated */
            m_floor->SetChanged();
            controller.publish_event(foraging_controller::BLOCK_FOUND);
            break;
          }
        } /* for(i..) */
      }
    }
    ++i;
  } /* for(it..) */

  /* Output stuff to file */
  m_ofile << GetSpace().GetSimulationClock() << "\t"
          << m_total_collected_blocks << "\t"
          << n_resting << "\t"
          << n_exploring << "\t"
          << n_returning << "\t"
          << n_avoiding << std::endl;
}
using namespace argos;
REGISTER_LOOP_FUNCTIONS(foraging_loop_functions, "foraging_loop_functions")

NS_END(fordyca);
