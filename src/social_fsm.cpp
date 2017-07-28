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
  if (mc_params.times.min_rested > m_state.time_rested &&
      m_pcRNG->Uniform(mc_params.prob_range) < m_state.rest_to_explore_prob) {
    internal_event(ST_EXPLORE);
  }

  /* continue resting */
  ++m_state.time_rested;
  /* Be sure not to send the last exploration result multiple times */
  if (m_state.time_rested == 1) {
    pr_raba_->SetData(0, LAST_EXPLORATION_NONE);
  }
  /*
   * Social rule: listen to what other people have found and modify
   * probabilities accordingly
   */
  const argos::CCI_RangeAndBearingSensor::TReadings& tPackets = m_sensors.range_and_bearing();
  for(size_t i = 0; i < tPackets.size(); ++i) {
    switch(tPackets[i].Data[0]) {
      case LAST_EXPLORATION_SUCCESSFUL: {
        m_state.rest_to_explore_prob += mc_params.deltas.social_rule_rest_to_explore;
        m_state.prob_range.TruncValue(m_state.rest_to_explore_prob);
        m_state.explore_to_rest_prob -= mc_params.deltas.social_rule_explore_to_rest;;
        m_state.prob_range.TruncValue(m_state.explore_to_rest_prob);
        break;
      }
      case LAST_EXPLORATION_UNSUCCESSFUL: {
        m_state.explore_to_rest_prob += m_state.SocialRuleExploreToRestDeltaProb;
        m_state.prob_range.TruncValue(m_state.explore_to_rest_prob);
        m_state.RestToExploreProb -= m_state.social_rule_rest_to_explore_prob_delta_;
        m_state.prob_range.TruncValue(m_state.RestToExploreProb);
        break;
      }
    }
  }
}

EXIT_DEFINE(social_fsm, exit_rest) {
  m_state.time_rested = 0;
}

ENTRY_DEFINE(social_fsm, entry_rest, fsm::no_event_data) {
  m_actuators.leds_set_color(argos::CColor::BLACK);
}

ENTRY_DEFINE(social_fsm, entry_explore, fsm::no_event_data) {
  m_actuators.leds_set_color(argos::CColor::MAGENTA);
}

STATE_DEFINE(social_fsm, explore, fsm::no_event_data) {
  /*
   * We transition to the 'return to nest' state in two situations:
   *
   * 1. If we have a food item
   * 2. If we have not found a food item for some time; in this case, the switch
   *    is probabilistic.
   */

  /* First condition: have we found a food item? */
  if (m_state.food_data.has_item) {
    external_event(ST_EXPLORE_SUCCESS);
  }

  /*
   * Second condition: we probabilistically switch to 'return to
   * nest' if we have been wandering for some time and found nothing.
   */
   if (m_state.time_exploring_unsuccessfully > mc_params.times.max_unsuccessful_explore) {
    if (m_pcRNG->Uniform(m_state.prob_range) < m_state.explore_to_rest_prob) {
      external_event(ST_EXPLORE_FAIL);
    }
    /* Apply the food rule, increasing explore_to_rest_prob and
     * decreasing RestToExploreProb */
    m_state.explore_to_rest_prob += mc_params.deltas.food_rule_explore_to_rest;
    m_state.prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.rest_to_explore_prob -= mc_params.deltas.food_rule_rest_to_explore;
    m_state.prob_range.TruncValue(m_state.rest_to_explore_prob);
  }

  /* perform the actual exploration */
  ++m_state.time_exploring_unsuccessfully;

  /* Get the diffusion vector to perform obstacle avoidance */
  struct collision_event_data collision_data;
  if (m_sensors.calc_diffusion_vector(collision_data.vector)) {
    external_event(ST_COLLISION_AVOIDANCE, &collision_data);
  }

  /* No obstacles nearby--use the diffusion vector only to set speeds */
  m_actuators.set_wheel_speeds(m_sWheelTurningParams.max_speed * cDiffusion);
}

EXIT_DEFINE(social_fsm, exit_explore) {
  m_state.time_exploring_unsuccessfully = 0;
  m_state.time_search_for_place_in_nest = 0;
}

STATE_DEFINE(social_fsm, collision_avoidance, struct collision_event_data) {
  while (calc_diffusion_vector(&data->vector)) {
    data->vector = -data->vector.Normalize();

    /* Use the diffusion vector only */
    SetWheelSpeedsFromVector(data->vector);
    /*
     * Collision avoidance happened, increase explore_to_rest_prob and decrease
     * RestToExploreProb
     */
    m_state.explore_to_rest_prob += mc_params.deltas.collision_rule_explore_to_rest;
    m_state.prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.RestToExploreProb -= mc_params.deltas.collision_rule_explore_to_rest;
    m_state.prob_range.TruncValue(m_state.rest_to_explore_prob);
  } /* while() */

  internal_event(last_state());
}

ENTRY_DEFINE(social_fsm, entry_collision_avoidance, fsm::no_event_data) {
  actuators.leds_set_color(argos::CColor::RED);
}

STATE_DEFINE(social_fsm, return_to_nest, fsm::no_event_data) {
  /* Read stuff from the ground sensor */
  const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
  /*
   * You can say whether you are in the nest by checking the ground sensor
   * placed close to the wheel motors. It returns a value between 0 and 1.  It
   * is 1 when the robot is on a white area, it is 0 when the robot is on a
   * black area and it is around 0.5 when the robot is on a gray area.
   *
   * The foot-bot has 4 sensors like this, two in the front (corresponding to
   * readings 0 and 1) and two in the back (corresponding to reading 2 and 3).
   * Here we want the back sensors (readings 2 and 3) to tell us whether we are
   * on gray: if so, the robot is completely in the nest, otherwise it's
   * outside.
   */
   if (tGroundReads[2].Value > 0.25f && tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f && tGroundReads[3].Value < 0.75f) {
     internal_event(ST_SEARCH_FOR_SPOT_IN_NEST);
   }
}

STATE_DEFINE(social_fsm, search_for_spot_in_nest, fsm::no_event_data) {
      /* Have we looked for a place long enough? */
  if (m_state.time_search_for_place_in_nest_ > mc_params.times.min_search_for_place_in_nest) {
         /* Yes, stop the wheels... */
         pc_wheels_->SetLinearVelocity(0.0f, 0.0f);
         /* Tell people about the last exploration attempt */
         pr_raba_->SetData(0, m_eLastExplorationResult);
         /* ... and switch to state 'resting' */
         internal_event(ST_REST);
  }
  /* No, keep looking */
  ++m_state.time_search_for_place_in_nest;
  m_actuators.set_wheel_speeds(
      m_actuators.max_wheel_speed() * m_sensors.calc_diffusion_vector() +
      m_actuators.max_wheel_speed() * m_sensors.calc_vector_to_light());
}

ENTRY_DEFINE(social_fsm, entry_search_for_spot_in_nest, fsm::no_event_data) {
  m_actuators.leds_set_color(argos::CColor::YELLOW);
}

EXIT_DEFINE(social_fsm, entry_search_for_spot_in_nest, fsm::no_event_data) {
  m_eLastExplorationResult = LAST_EXPLORATION_NONE;
  m_state.time_search_for_place_in_nest_ = 0;
}

STATE_DEFINE(social_fsm, explore_success, fsm::no_event_data) {
  m_actuators.leds_set_color(argos::CColor::GREEN);

  /*
   * Apply the food rule, decreasing explore_to_rest_prob and increasing
   * RestToExploreProb
   */
  m_state.explore_to_rest_prob -= m_state.food_rule_explore_to_rest_prob_delta_;
  m_state.prob_range.TruncValue(m_state.explore_to_rest_prob);
  m_state.RestToExploreProb += m_state.food_rule_rest_to_explre_prob_delta_;
  m_state.prob_range.TruncValue(m_state.RestToExploreProb);

  /* Store the result of the expedition */
  m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}

STATE_DEFINE(social_fsm, explore_fail, fsm::no_event_data) {
  m_actuators.leds_set_color(argos::CColor::ORANGE);

  /* Store the result of the expedition */
  m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}

STATE_DEFINE(social_fsm, in_nest, fsm::no_event_data) {
  struct collision_event_data data;
  if (calc_diffusion_vector(&data.vector)) {
    external_event(ST_COLLISION_AVOIDANCE, &data);
  }
  /*
   * The vector returned by calc_vector_to_light() points to
   * the light. Thus, the minus sign is because we want to go away
   * from the light.
   */
  m_actuators.set_wheels_speeds(
      m_actuators.max_wheel_speed() * cDiffusion -
      m_actuators.max_wheel_speed() * 0.25f * m_sensors.calc_vector_to_light());
}

NS_END(fordyca);
