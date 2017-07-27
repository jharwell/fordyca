/**
 * @file social_foraging_fsm.cpp
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
#include "fordyca/social_foraging_fsm.hpp"

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
void social_foraging_fsm::event_explore(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_EXPLORE,         /* resting */
    EVENT_IGNORED,      /* explore */
    CANNOT_HAPPEN,      /* explore fail */
    CANNOT_HAPPEN,      /* explore success */
    EVENT_IGNORED,      /* return to nest */
    ST_COLLISION_AVOIDANCE, /* collision avoidance */
    ST_IN_NEST,         /* ST_EXPLORE */
  };
  external_event(kTRANSITIONS[current_state()], NULL);
}

void social_foraging_fsm::event_explore_success(void) {
  static const uint8_t kTRANSITIONS[] = {
    CANNOT_HAPPEN,      /* was resting */
    ST_EXPLORE_SUCCESS, /* was exploring */
    EVENT_IGNORED,      /* another success? */
    CANNOT_HAPPEN,      /* already got success... */
    CANNOT_HAPPEN,      /* can't have exploration success while returning to nest */
  };
  external_event(kTRANSITIONS[current_state()], NULL);
}

void social_foraging_fsm::event_explore_fail(void) {
  static const uint8_t kTRANSITIONS[] = {
    CANNOT_HAPPEN,      /* was resting */
    ST_EXPLORE_FAIL, /* was exploring */
    EVENT_IGNORED,      /* another success? */
    CANNOT_HAPPEN,      /* already got success... */
    CANNOT_HAPPEN,      /* can't have exploration success while returning to nest */
  };
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
STATE_DEFINE(social_foraging_fsm, rest, fsm::no_event_data) {
  /*
   * If we have stayed here enough, probabilistically switch to 'exploring'
   */
     internal_event(ST_EXPLORE);

     /* continue resting */
     ++m_sStateData.TimeRested;
     /* Be sure not to send the last exploration result multiple times */
     if(m_sStateData.TimeRested == 1) {
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
           m_sStateData.RestToExploreProb += m_sStateData.SocialRuleRestToExploreDeltaProb;
           m_sStateData.prob_range.TruncValue(m_sStateData.RestToExploreProb);
           m_sStateData.explore_to_rest_prob -= m_sStateData.SocialRuleExploreToRestDeltaProb;
           m_sStateData.prob_range.TruncValue(m_sStateData.explore_to_rest_prob);
           break;
         }
         case LAST_EXPLORATION_UNSUCCESSFUL: {
           m_sStateData.explore_to_rest_prob += m_sStateData.SocialRuleExploreToRestDeltaProb;
           m_sStateData.prob_range.TruncValue(m_sStateData.explore_to_rest_prob);
           m_sStateData.RestToExploreProb -= m_sStateData.SocialRuleRestToExploreDeltaProb;
           m_sStateData.prob_range.TruncValue(m_sStateData.RestToExploreProb);
           break;
         }
       }
     }
}

EXIT_DEFINE(social_foraging_fsm, exit_rest) {
  pc_leds_->SetAllColors(CColor::GREEN);
  m_sStateData.State = SStateData::STATE_EXPLORING;
  m_sStateData.TimeRested = 0;
}

GUARD_DEFINE(social_foraging_fsm, exit_rest, fsm::no_event_data) {
  if (m_sStateData.TimeRested > m_sStateData.MinimumRestingTime) {
    return true;
  }
  return false;
}

STATE_DEFINE(social_foraging_fsm, explore, fsm::no_event_data) {
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
   else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
     if (m_pcRNG->Uniform(m_sStateData.prob_range) < m_sStateData.explore_to_rest_prob) {
       external_event(ST_EXPLORE_FAIL);
     }
     /* Apply the food rule, increasing explore_to_rest_prob and
      * decreasing RestToExploreProb */
     m_sStateData.explore_to_rest_prob += m_sStateData.FoodRuleExploreToRestDeltaProb;
     m_sStateData.prob_range.TruncValue(m_sStateData.explore_to_rest_prob);
     m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
     m_sStateData.prob_range.TruncValue(m_sStateData.RestToExploreProb);
   }

   /* perform the actual exploration */
   ++m_sStateData.TimeExploringUnsuccessfully;

   /* Get the diffusion vector to perform obstacle avoidance */
   collision_event_data data;
   if (calc_diffusion_vector(&data.vector)) {
     external_event(ST_COLLISION_AVOIDANCE, &data);
   }

   if (in_nest()) {
     internal_event(ST_IN_NEST);
   }

   /* No obstacles nearby--use the diffusion vector only to set speeds */
   SetWheelSpeedsFromVector(m_sWheelTurningParams.max_speed * cDiffusion);
}
void in_nest() {
   /* Reset state flags */
   m_sStateData.InNest = false;
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
      m_sStateData.InNest = true;
   }
}

EXIT_DEFINE(social_foraging_fsm, exit_explore) {
  m_sStateData.TimeExploringUnsuccessfully = 0;
  m_sStateData.TimeSearchingForPlaceInNest = 0;
  pc_leds_->SetAllColors(CColor::BLUE);
  m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
}

STATE_DEFINE(social_foraging_fsm, collision_avoidance, collision_event_data) {
  calc_diffusion_vector(&data->vector);
  data->vector = -data->vector.Normalize();

  /* Use the diffusion vector only */
  SetWheelSpeedsFromVector(data->vector);

  /*
   * Collision avoidance happened, increase explore_to_rest_prob and decrease
   * RestToExploreProb
   */
  m_sStateData.explore_to_rest_prob += m_sStateData.CollisionRuleExploreToRestDeltaProb;
  m_sStateData.prob_range.TruncValue(m_sStateData.explore_to_rest_prob);
  m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
  m_sStateData.prob_range.TruncValue(m_sStateData.RestToExploreProb);

  internal_event(last_state());
}

GUARD_DEFINE(social_foraging_fsm, guard_collision_avoidance, collision_event_data) {
  if (calc_diffusion_vector(&data->vector)) {
    return false;
  }
  return true;
}

STATE_DEFINE(social_foraging_fsm, return_to_nest, fsm::no_event_data) {
     /* As soon as you get to the nest, switch to 'resting' */
   UpdateState();
   /* Are we in the nest? */
   if(m_sStateData.InNest) {
      /* Have we looked for a place long enough? */
      if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
         /* Yes, stop the wheels... */
         pc_wheels_->SetLinearVelocity(0.0f, 0.0f);
         /* Tell people about the last exploration attempt */
         pr_raba_->SetData(0, m_eLastExplorationResult);
         /* ... and switch to state 'resting' */
         pc_leds_->SetAllColors(CColor::RED);
         m_sStateData.State = SStateData::STATE_RESTING;
         m_sStateData.TimeSearchingForPlaceInNest = 0;
         m_eLastExplorationResult = LAST_EXPLORATION_NONE;
         return;
      }
      else {
         /* No, keep looking */
         ++m_sStateData.TimeSearchingForPlaceInNest;
      }
   }
   else {
      /* Still outside the nest */
      m_sStateData.TimeSearchingForPlaceInNest = 0;
   }
   /* Keep going */
   bool bCollision;
   SetWheelSpeedsFromVector(
      m_sWheelTurningParams.max_speed * calc_diffusion_vector(bCollision) +
      m_sWheelTurningParams.max_speed * calc_vector_to_light());
}

STATE_DEFINE(social_foraging_fsm, explore_success, fsm::no_event_data) {
  /*
   * Apply the food rule, decreasing explore_to_rest_prob and increasing
   * RestToExploreProb
   */
  m_sStateData.explore_to_rest_prob -= m_sStateData.FoodRuleExploreToRestDeltaProb;
  m_sStateData.prob_range.TruncValue(m_sStateData.explore_to_rest_prob);
  m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
  m_sStateData.prob_range.TruncValue(m_sStateData.RestToExploreProb);
  /* Store the result of the expedition */
  m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;

  internal_event(ST_RETURN_TO_NEST);
}

STATE_DEFINE(social_foraging_fsm, explore_fail, fsm::no_event_data) {
    /* Store the result of the expedition */
    m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
    /* Switch to 'return to nest' */
    bReturnToNest = true;
  internal_event(ST_RETURN_TO_NEST);

}

STATE_DEFINE(social_foraging_fsm, in_nest, fsm::no_event_data) {
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
