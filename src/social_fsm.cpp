/**
 * @file social_fsm.cpp
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
#include "fordyca/social_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Events
 ******************************************************************************/
void social_fsm::event_explore(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_EXPLORE,         /* resting */
    EVENT_IGNORED,      /* explore */
    EVENT_IGNORED,      /* explore fail */
    EVENT_IGNORED,     /* explore success */
    EVENT_IGNORED,      /* return to nest */
    ST_COLLISION_AVOIDANCE, /* collision avoidance */
  };
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
STATE_DEFINE(social_fsm, rest, fsm::no_event_data) {
  /*
   * If we have stayed here enough, probabilistically switch to 'exploring'
   */
  if (config_.times.min_rested > state_.time_rested &&
      m_pcRNG->Uniform(config_.prob_range) < state_.rest_to_explore_prob) {
    internal_event(ST_EXPLORE);
  }

     /* continue resting */
     ++state_.time_rested;
     /* Be sure not to send the last exploration result multiple times */
     if (state_.time_rested == 1) {
       pr_raba_->SetData(0, LAST_EXPLORATION_NONE);
     }
     /*
      * Social rule: listen to what other people have found and modify
      * probabilities accordingly
      */
     const CCI_RangeAndBearingSensor::TReadings& tPackets = rabs_->GetReadings();
     for(size_t i = 0; i < tPackets.size(); ++i) {
       switch(tPackets[i].Data[0]) {
         case LAST_EXPLORATION_SUCCESSFUL: {
           state_.RestToExploreProb += state_.social_rule_rest_to_explore_prob_delta_;
           state_.prob_range.TruncValue(state_.RestToExploreProb);
           state_.explore_to_rest_prob -= state_.SocialRuleExploreToRestDeltaProb;
           state_.prob_range.TruncValue(state_.explore_to_rest_prob);
           break;
         }
         case LAST_EXPLORATION_UNSUCCESSFUL: {
           state_.explore_to_rest_prob += state_.SocialRuleExploreToRestDeltaProb;
           state_.prob_range.TruncValue(state_.explore_to_rest_prob);
           state_.RestToExploreProb -= state_.social_rule_rest_to_explore_prob_delta_;
           state_.prob_range.TruncValue(state_.RestToExploreProb);
           break;
         }
       }
     }
}

EXIT_DEFINE(social_fsm, exit_rest) {
  state_.time_rested = 0;
}

ENTRY_DEFINE(social_fsm, entry_rest, fsm::no_event_data) {
  pc_leds_->SetAllColors(argos::CColor::BLACK);
}

ENTRY_DEFINE(social_fsm, entry_explore, fsm::no_event_data) {
  pc_leds_->SetAllColors(argos::CColor::MAGENTA);
}

STATE_DEFINE(social_fsm, explore, fsm::no_event_data) {
     /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */
   bool bReturnToNest(false);
   /*
    * Test the first condition: have we found a food item?  NOTE: the food data
    * is updated by the loop functions, so here we just need to read it
    *
    * TODO: Put this stuff in a guard condition.
    */
   if(m_sFoodData.has_item_) {
     external_event(ST_EXPLORE_SUCCESS);
   }
   /* Test the second condition: we probabilistically switch to 'return to
    * nest' if we have been wandering for some time and found nothing */
   else if(state_.time_exploring_unsuccessfully > config_.times.max_unsuccessful_explore) {
     if (m_pcRNG->Uniform(state_.prob_range) < state_.explore_to_rest_prob) {
       external_event(ST_EXPLORE_FAIL);
     }
     /* Apply the food rule, increasing explore_to_rest_prob and
      * decreasing RestToExploreProb */
     state_.explore_to_rest_prob += config_.deltas.food_rule_explore_to_rest;
     state_.prob_range.TruncValue(state_.explore_to_rest_prob);
     state_.RestToExploreProb -= config_.deltas.food_rule_rest_to_explore;
     state_.prob_range.TruncValue(state_.rest_to_explore_prob);
   }

   /* perform the actual exploration */
   ++state_.time_exploring_unsuccessfully;

   /* Get the diffusion vector to perform obstacle avoidance */
   struct collision_event_data collision_data;
   if (calc_diffusion_vector(&collision_data.vector)) {
     external_event(ST_COLLISION_AVOIDANCE, &collision_data);
   }

   if (in_nest()) {
     internal_event(ST_IN_NEST);
   }

   /* No obstacles nearby--use the diffusion vector only to set speeds */
   SetWheelSpeedsFromVector(m_sWheelTurningParams.max_speed * cDiffusion);
}
void in_nest() {
   /* Reset state flags */
   state_.InNest = false;
   /* Read stuff from the ground sensor */
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
   /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray
    * area.
    * The foot-bot has 4 sensors like this, two in the front
    * (corresponding to readings 0 and 1) and two in the back
    * (corresponding to reading 2 and 3).  Here we want the back sensors
    * (readings 2 and 3) to tell us whether we are on gray: if so, the
    * robot is completely in the nest, otherwise it's outside.
    */
   if(tGroundReads[2].Value > 0.25f &&
      tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f &&
      tGroundReads[3].Value < 0.75f) {
      state_.InNest = true;
   }
}

EXIT_DEFINE(social_fsm, exit_explore) {
  state_.time_exploring_unsuccessfully_ = 0;
  state_.time_search_for_place_in_nest_ = 0;
}

STATE_DEFINE(social_fsm, collision_avoidance, collision_event_data) {
  calc_diffusion_vector(&data->vector);
  data->vector = -data->vector.Normalize();

  /* Use the diffusion vector only */
  SetWheelSpeedsFromVector(data->vector);

  /*
   * Collision avoidance happened, increase explore_to_rest_prob and decrease
   * RestToExploreProb
   */
  state_.explore_to_rest_prob += state_.collision_rule_explore_to_resta_prob_delta_;
  state_.prob_range.TruncValue(state_.explore_to_rest_prob);
  state_.RestToExploreProb -= state_.collision_rule_explore_to_resta_prob_delta_;
  state_.prob_range.TruncValue(state_.RestToExploreProb);

  internal_event(last_state());
}

GUARD_DEFINE(social_fsm, guard_collision_avoidance, collision_event_data) {
  if (calc_diffusion_vector(&data->vector)) {
    return false;
  }
  return true;
}

STATE_DEFINE(social_fsm, return_to_nest, fsm::no_event_data) {
     /* As soon as you get to the nest, switch to 'resting' */
   UpdateState();
   /* Are we in the nest? */
   if(state_.InNest) {
      /* Have we looked for a place long enough? */
      if(state_.time_search_for_place_in_nest_ > state_.MinimumSearchForPlaceInNestTime) {
         /* Yes, stop the wheels... */
         pc_wheels_->SetLinearVelocity(0.0f, 0.0f);
         /* Tell people about the last exploration attempt */
         pr_raba_->SetData(0, m_eLastExplorationResult);
         /* ... and switch to state 'resting' */
         pc_leds_->SetAllColors(CColor::RED);
         state_.State = SStateData::STATE_RESTING;
         state_.time_search_for_place_in_nest_ = 0;
         m_eLastExplorationResult = LAST_EXPLORATION_NONE;
         return;
      }
      else {
         /* No, keep looking */
         ++state_.time_search_for_place_in_nest_;
      }
   }
   else {
      /* Still outside the nest */
      state_.time_search_for_place_in_nest_ = 0;
   }
   /* Keep going */
   bool bCollision;
   SetWheelSpeedsFromVector(
      m_sWheelTurningParams.max_speed * calc_diffusion_vector(bCollision) +
      m_sWheelTurningParams.max_speed * calc_vector_to_light());
}

STATE_DEFINE(social_fsm, explore_success, fsm::no_event_data) {
  pc_leds_->SetAllColors(argos::CColor::GREEN);

  /*
   * Apply the food rule, decreasing explore_to_rest_prob and increasing
   * RestToExploreProb
   */
  state_.explore_to_rest_prob -= state_.food_rule_explore_to_rest_prob_delta_;
  state_.prob_range.TruncValue(state_.explore_to_rest_prob);
  state_.RestToExploreProb += state_.food_rule_rest_to_explre_prob_delta_;
  state_.prob_range.TruncValue(state_.RestToExploreProb);

  /* Store the result of the expedition */
  m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}

STATE_DEFINE(social_fsm, explore_fail, fsm::no_event_data) {
  pc_leds_->SetAllColors(argos::CColor::ORANGE);

  /* Store the result of the expedition */
  m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}

STATE_DEFINE(social_fsm, in_nest, fsm::no_event_data) {
  collision_event_data data;
  if (calc_diffusion_vector(&data.vector)) {
    external_event(ST_COLLISION_AVOIDANCE, &data);
  }
  /*
   * The vector returned by calc_vector_to_light() points to
   * the light. Thus, the minus sign is because we want to go away
   * from the light.
   */
  SetWheelSpeedsFromVector(
      m_sWheelTurningParams.max_speed * cDiffusion -
      m_sWheelTurningParams.max_speed * 0.25f * calc_vector_to_light());
}

NS_END(fordyca);
