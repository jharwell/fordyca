/**
 * @file social_foraging.cpp
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
#include "fordyca/footbot_foraging.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace foraging_controller {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
SFoodData::SFoodData() :
   has_item_(false),
   curr_item_idx_(0),
   cum_items_(0) {}

SDiffusionParams::SDiffusionParams() :
    go_straight_angle_range(CRadians(-1.0f), CRadians(1.0f)) {}

void SFoodData::Reset() {
   has_item_ = false;
   curr_item_idx = 0;
   cum_items_ = 0;
}

/****************************************/
/****************************************/


void SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cgo_straight_angle_rangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cgo_straight_angle_rangeDegrees);
      go_straight_angle_range.Set(ToRadians(cgo_straight_angle_rangeDegrees.GetMin()),
                               ToRadians(cgo_straight_angle_rangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      hard_turn_threshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      soft_turn_threshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", max_speed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

SStateData::SStateData() :
   prob_range(0.0f, 1.0f) {}

void SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "initial_rest_to_explore_prob", initial_rest_to_explor_prob);
      GetNodeAttribute(t_node, "initial_explore_to_rest_prob", initial_explore_to_rest_prob);
      GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", food_rule_explore_to_rest_prob_delta_);
      GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", food_rule_rest_to_explre_prob_delta_);
      GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", collision_rule_explore_to_resta_prob_delta_);
      GetNodeAttribute(t_node, "social_rule_rest_to_explore_delta_prob", social_rule_rest_to_explore_prob_delta_);
      GetNodeAttribute(t_node, "social_rule_explore_to_rest_delta_prob", SocialRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
      GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = initial_rest_to_explor_prob;
   explore_to_rest_prob = initial_explore_to_rest_prob;
   time_exploring_unsuccessfully_ = 0;
   /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
   time_rested_ = MinimumRestingTime;
   time_search_for_place_in_nest_ = 0;
}

/****************************************/
/****************************************/

foraging_base() :
   pc_wheels_(NULL),
   pc_leds_(NULL),
   pr_raba_(NULL),
   m_rabs(NULL),
   m_pcProximity(NULL),
   m_light(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL) {}

/****************************************/
/****************************************/

void Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      pc_wheels_    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      pc_leds_      = GetActuator<CCI_LEDsActuator                >("leds"                 );
      pr_raba_      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
      m_rabs      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_light     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      m_pcGround    = GetSensor  <CCI_FootBotMotorGroundSensor    >("footbot_motor_ground" );
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      state_.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \"" << GetId() << "\"", ex);
   }
   /*
    * Initialize other stuff
    */
   /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CRandom::CreateRNG("argos");
   Reset();
}

/****************************************/
/****************************************/

void ControlStep() {
  UpdateState();
   switch(state_.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         ReturnToNest();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void Reset() {
   /* Reset robot state */
   state_.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   pc_leds_->SetAllColors(CColor::RED);
   /* Clear up the last exploration result */
   m_eLastExplorationResult = LAST_EXPLORATION_NONE;
   pr_raba_->ClearData();
   pr_raba_->SetData(0, LAST_EXPLORATION_NONE);
}

/****************************************/
/****************************************/

void UpdateState() {
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

/****************************************/
/****************************************/

CVector2 calc_vector_to_light() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_light->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

/****************************************/
/****************************************/


/****************************************/
/****************************************/

/****************************************/
/****************************************/

void Rest() {
   /* If we have stayed here enough, probabilistically switch to
    * 'exploring' */
   if(state_.time_rested_ > state_.MinimumRestingTime &&
      m_pcRNG->Uniform(state_.prob_range) < state_.RestToExploreProb) {
      pc_leds_->SetAllColors(CColor::GREEN);
      state_.State = SStateData::STATE_EXPLORING;
      state_.time_rested_ = 0;
   }
   else {
      ++state_.time_rested_;
      /* Be sure not to send the last exploration result multiple times */
      if(state_.time_rested_ == 1) {
         pr_raba_->SetData(0, LAST_EXPLORATION_NONE);
      }
      /*
       * Social rule: listen to what other people have found and modify
       * probabilities accordingly
       */
      const CCI_RangeAndBearingSensor::TReadings& tPackets = m_rabs->GetReadings();
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
}

/****************************************/
/****************************************/

void Explore() {
   /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */
   bool bReturnToNest(false);
   /*
    * Test the first condition: have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    */
   if(m_sFoodData.has_item_) {
      /* Apply the food rule, decreasing explore_to_rest_prob and increasing
       * RestToExploreProb */
      state_.explore_to_rest_prob -= state_.food_rule_explore_to_rest_prob_delta_;
      state_.prob_range.TruncValue(state_.explore_to_rest_prob);
      state_.RestToExploreProb += state_.food_rule_rest_to_explre_prob_delta_;
      state_.prob_range.TruncValue(state_.RestToExploreProb);
      /* Store the result of the expedition */
      m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
      /* Switch to 'return to nest' */
      bReturnToNest = true;
   }
   /* Test the second condition: we probabilistically switch to 'return to
    * nest' if we have been wandering for some time and found nothing */
   else if(state_.time_exploring_unsuccessfully_ > state_.MinimumUnsuccessfulExploreTime) {
      if (m_pcRNG->Uniform(state_.prob_range) < state_.explore_to_rest_prob) {
         /* Store the result of the expedition */
         m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
         /* Switch to 'return to nest' */
         bReturnToNest = true;
      }
      else {
         /* Apply the food rule, increasing explore_to_rest_prob and
          * decreasing RestToExploreProb */
         state_.explore_to_rest_prob += state_.food_rule_explore_to_rest_prob_delta_;
         state_.prob_range.TruncValue(state_.explore_to_rest_prob);
         state_.RestToExploreProb -= state_.food_rule_rest_to_explre_prob_delta_;
         state_.prob_range.TruncValue(state_.RestToExploreProb);
      }
   }
   /* So, do we return to the nest now? */
   if(bReturnToNest) {
      /* Yes, we do! */
      state_.time_exploring_unsuccessfully_ = 0;
      state_.time_search_for_place_in_nest_ = 0;
      pc_leds_->SetAllColors(CColor::BLUE);
      state_.State = SStateData::STATE_RETURN_TO_NEST;
   }
   else {
      /* No, perform the actual exploration */
      ++state_.time_exploring_unsuccessfully_;
      UpdateState();
      /* Get the diffusion vector to perform obstacle avoidance */
      bool bCollision;
      CVector2 cDiffusion = calc_diffusion_vector(bCollision);
      /* Apply the collision rule, if a collision avoidance happened */
      if(bCollision) {
         /* Collision avoidance happened, increase explore_to_rest_prob and
          * decrease RestToExploreProb */
         state_.explore_to_rest_prob += state_.collision_rule_explore_to_resta_prob_delta_;
         state_.prob_range.TruncValue(state_.explore_to_rest_prob);
         state_.RestToExploreProb -= state_.collision_rule_explore_to_resta_prob_delta_;
         state_.prob_range.TruncValue(state_.RestToExploreProb);
      }
      /*
       * If we are in the nest, we combine antiphototaxis with obstacle
       * avoidance
       * Outside the nest, we just use the diffusion vector
       */
      if(state_.InNest) {
         /*
          * The vector returned by calc_vector_to_light() points to
          * the light. Thus, the minus sign is because we want to go away
          * from the light.
          */
         SetWheelSpeedsFromVector(
            m_sWheelTurningParams.max_speed * cDiffusion -
            m_sWheelTurningParams.max_speed * 0.25f * calc_vector_to_light());
      }
      else {
         /* Use the diffusion vector only */
         SetWheelSpeedsFromVector(m_sWheelTurningParams.max_speed * cDiffusion);
      }
   }
}


/****************************************/
/****************************************/

void ReturnToNest() {
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

/*
 * This statement notifies ARGoS of the existence of the controller.  It binds
 * the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this
 * controller.  When ARGoS reads that string in the XML file, it knows which
 * controller class to instantiate.  See also the XML configuration files for an
 * example of how this is used.
 */
REGISTER_CONTROLLER(foraging_base, "footbot_foraging_controller")
} /* namespace foraging_controller */
