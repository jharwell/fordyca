/**
 * @file social_loop_functions.cpp
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
#include "fordyca/social_loop_functions.hpp"
#include "fordyca/social_foraging_controller.hpp"
#include "fordyca/food_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
social_loop_functions::social_loop_functions(void) :
    m_arena_x(-0.9f, 1.7f),
    m_arena_y(-1.7f, 1.7f),
    m_food_pos(),
    m_floor(NULL),
    m_rng(NULL),
    m_ofname(),
    m_ofile(),
    m_uncollected_food(0),
    m_energy(0),
    m_energy_per_moving_robot(1),
    m_food_params(),
    m_parser() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void social_loop_functions::Init(argos::TConfigurationNode& t_node) {
  argos::TConfigurationNode& foraging = argos::GetNode(t_node, "foraging");
  m_parser.add_category("food", new food_param_parser());
  m_parser.parse_all(foraging);

  m_floor = &GetSpace().GetFloorEntity();
  m_rng = argos::CRandom::CreateRNG("argos");

  /*
   * Distribute food items uniformly in the arena.
   */
  for (size_t i = 0; i < m_food_params.n_items; ++i) {
    m_food_pos.push_back(
        argos::CVector2(m_rng->Uniform(m_arena_x),
                        m_rng->Uniform(m_arena_y)));
  } /* for(i..) */

  /* Open output file and truncate */
  argos::GetNodeAttribute(foraging, "output", m_ofname);
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);

  m_ofile << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;

  /* Get energy loss per moving robot */
  argos::GetNodeAttribute(foraging, "energy_per_moving_robot", m_energy_per_moving_robot);
}

void social_loop_functions::Reset() {
  m_uncollected_food = 0;
  m_energy = 0;
  m_ofile.close();
  m_ofile.open(m_ofname.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_ofile << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;

  for (size_t i = 0; i < m_food_pos.size(); ++i) {
    m_food_pos[i].Set(m_rng->Uniform(m_arena_x),
                      m_rng->Uniform(m_arena_y));
  } /* for(i..) */
}

void social_loop_functions::Destroy() {
  m_ofile.close();
}

argos::CColor social_loop_functions::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  if (c_position_on_plane.GetX() < -1.0f) {
    return argos::CColor::GRAY50;
  }
  for (size_t i = 0; i < m_food_pos.size(); ++i) {
    if ((c_position_on_plane - m_food_pos[i]).SquareLength() < m_food_params.square_radius) {
      return argos::CColor::BLACK;
    }
  }
  return argos::CColor::WHITE;
} /* get_floor_color() */


/****************************************/
/****************************************/

void social_loop_functions::PreStep() {
  /* Logic to pick and drop food items */
  /*
   * If a robot is in the nest, drop the food item
   * If a robot is on a food item, pick it
   * Each robot can carry only one food item per time
   */
  uint unWalkingFBs = 0;
  uint unRestingFBs = 0;
  /* Check whether a robot is on a food item */
  argos::CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
    /* Get handle to foot-bot entity and controller */
    argos::CFootBotEntity& cFootBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    social_foraging_controller& cController = dynamic_cast<social_foraging_controller&>(cFootBot.GetControllableEntity().GetController());
    /* Count how many foot-bots are in which state */
    if (!cController.is_resting()) {
      ++unWalkingFBs;
    } else {
      ++unRestingFBs;
    }
    /* Get the position of the foot-bot on the ground as a CVector2 */
    argos::CVector2 cPos;
    cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
             cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    /* Get food data */
    social_foraging_controller::food_data& food_stats = cController.get_food_data();
    /* The foot-bot has a food item */
    if (food_stats.has_item) {
      /* Check whether the foot-bot is in the nest */
      if(cPos.GetX() < -1.0f) {
        /* Place a new food item on the ground */
        m_food_pos[food_stats.curr_item_idx].Set(m_rng->Uniform(m_arena_x),
                                                 m_rng->Uniform(m_arena_y));
        /* Drop the food item */
        food_stats.has_item = false;
        food_stats.curr_item_idx = -1;
        ++food_stats.cum_items;
        m_energy += m_food_params.energy_per_item;
        ++m_uncollected_food;
        /* The floor texture must be updated */
        m_floor->SetChanged();
      }
    } else {
      /* The foot-bot has no food item */
      /* Check whether the foot-bot is out of the nest */
      if (cPos.GetX() > -1.0f) {
        /* Check whether the foot-bot is on a food item */
        for (size_t i = 0; i < m_food_pos.size(); ++i) {
          if((cPos - m_food_pos[i]).SquareLength() < m_food_params.square_radius) {
            /* If so, we move that item out of sight */
            m_food_pos[i].Set(100.0f, 100.f);
            /* The foot-bot is now carrying an item */
            food_stats.has_item = true;
            food_stats.curr_item_idx = i;
            /* The floor texture must be updated */
            m_floor->SetChanged();
            break;
          }
        } /* for(i..) */
      }
    }
  }
  /* Update energy expediture due to walking robots */
  m_energy -= unWalkingFBs * m_energy_per_moving_robot;
  /* Output stuff to file */
  m_ofile << GetSpace().GetSimulationClock() << "\t"
          << unWalkingFBs << "\t"
          << unRestingFBs << "\t"
          << m_uncollected_food << "\t"
          << m_energy << std::endl;
}
using namespace argos;
REGISTER_LOOP_FUNCTIONS(social_loop_functions, "social_loop_functions")

NS_END(fordyca);
