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
social_fsm::social_fsm(const struct social_fsm_params* params,
                       std::shared_ptr<rcppsw::common::er_server> server,
                       std::shared_ptr<sensor_manager> sensors,
                       std::shared_ptr<actuator_manager> actuators) :
    fsm::base_fsm(server, ST_MAX_STATES),
    rest(),
    explore(),
    explore_success(),
    explore_fail(),
    return_to_nest(),
    search_for_spot_in_nest(),
    collision_avoidance(),
    exit_explore(),
    exit_rest(),
    exit_collision_avoidance(),
    exit_search_for_spot_in_nest(),
    entry_explore(),
    entry_rest(),
    entry_collision_avoidance(),
    entry_search_for_spot_in_nest(),
    m_last_explore_res(LAST_EXPLORATION_NONE),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_prob_range(0.0f, 1.0f),
    m_state(),
    mc_params(params),
    m_sensors(sensors),
    m_actuators(actuators) {
  insmod("social_fsm");
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::DIAG);
    }

/*******************************************************************************
 * Events
 ******************************************************************************/
void social_fsm::event_explore(void) {
  DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    ST_EXPLORE,             /* resting */
    EVENT_IGNORED,          /* explore */
    EVENT_IGNORED,          /* explore success */
    EVENT_IGNORED,          /* explore fail */
    EVENT_IGNORED,          /* return to nest */
    EVENT_IGNORED,          /* leaving nest */
    EVENT_IGNORED,          /* search for spot in nest */
    ST_COLLISION_AVOIDANCE, /* collision avoidance */
  };
  VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void social_fsm::event_block_found(void) {
  DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    CANNOT_HAPPEN,          /* resting */
    ST_EXPLORE_SUCCESS,     /* explore */
    EVENT_IGNORED,          /* explore success */
    EVENT_IGNORED,          /* explore fail */
    EVENT_IGNORED,          /* return to nest */
    CANNOT_HAPPEN,          /* leaving to nest */
    EVENT_IGNORED,          /* search for spot in nest */
    ST_COLLISION_AVOIDANCE, /* collision avoidance */
        };
  VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void social_fsm::event_continue(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_REST,                    /* resting */
    ST_EXPLORE,                 /* explore */
    ST_EXPLORE_SUCCESS,         /* explore success */
    ST_EXPLORE_FAIL,            /* explore fail */
    ST_RETURN_TO_NEST,          /* return to nest */
    ST_LEAVING_NEST,            /* leaving to nest */
    ST_SEARCH_FOR_SPOT_IN_NEST, /* search for spot in nest */
    ST_COLLISION_AVOIDANCE      /* collision avoidance */
  };
  VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void social_fsm::event_entered_nest(void) {
  DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    CANNOT_HAPPEN,                  /* resting */
        EVENT_IGNORED,              /* explore */
        CANNOT_HAPPEN,              /* explore success */
        CANNOT_HAPPEN,              /* explore fail */
        ST_SEARCH_FOR_SPOT_IN_NEST, /* return to nest */
        CANNOT_HAPPEN,              /* leaving nest */
        EVENT_IGNORED,              /* search for spot in nest */
        ST_COLLISION_AVOIDANCE,     /* collision avoidance */
        };
  VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
STATE_DEFINE(social_fsm, rest, fsm::no_event_data) {
  ER_DIAG("Executing ST_REST");

  /*
   * If we have stayed here enough, probabilistically switch to 'exploring'
   */
  if (mc_params->times.min_rested > m_state.time_rested &&
      m_rng->Uniform(m_prob_range) < m_state.rest_to_explore_prob) {
    internal_event(ST_EXPLORE);
  }

  /* continue resting */
  ++m_state.time_rested;
  /* Be sure not to send the last exploration result multiple times */
  if (m_state.time_rested == 1) {
    m_actuators->set_raba_data(LAST_EXPLORATION_NONE);
  }
  /*
   * Social rule: listen to what other people have found and modify
   * probabilities accordingly
   */
  const argos::CCI_RangeAndBearingSensor::TReadings& tPackets = m_sensors->range_and_bearing();
  for (size_t i = 0; i < tPackets.size(); ++i) {
    switch (tPackets[i].Data[0]) {
      case LAST_EXPLORATION_SUCCESSFUL:
        m_state.rest_to_explore_prob += mc_params->deltas.social_rule_rest_to_explore;
        m_prob_range.TruncValue(m_state.rest_to_explore_prob);
        m_state.explore_to_rest_prob -= mc_params->deltas.social_rule_explore_to_rest;;
        m_prob_range.TruncValue(m_state.explore_to_rest_prob);
        break;
      case LAST_EXPLORATION_UNSUCCESSFUL:
        m_state.explore_to_rest_prob += mc_params->deltas.social_rule_explore_to_rest;
        m_prob_range.TruncValue(m_state.explore_to_rest_prob);
        m_state.rest_to_explore_prob -= mc_params->deltas.social_rule_rest_to_explore;
        m_prob_range.TruncValue(m_state.rest_to_explore_prob);
        break;
    } /* switch() */
  } /* for(i..) */
}
STATE_DEFINE(social_fsm, leaving_nest, fsm::no_event_data) {
  /*
   * The vector returned by calc_vector_to_light() points to
   * the light. Thus, the minus sign is because we want to go away
   * from the light.
   */
  argos::CVector2 diff_vector;
  m_sensors->calc_diffusion_vector(&diff_vector);
  m_actuators->set_wheel_speeds(
      m_actuators->max_wheel_speed() * diff_vector -
      m_actuators->max_wheel_speed() * 0.25f * m_sensors->calc_vector_to_light());
  if (!m_sensors->in_nest()) {
    internal_event(ST_EXPLORE);
  }
}
STATE_DEFINE(social_fsm, explore, fsm::no_event_data) {
  ER_DIAG("Executing ST_EXPlORE");

  /*
   * We transition to the 'return to nest' state in two situations:
   *
   * 1. If we have a food item, in which case we are notified by an external event.
   * 2. If we have not found a food item for some time; in this case, the switch
   *    is probabilistic.
   */

  /*
   * Second condition: we probabilistically switch to 'return to
   * nest' if we have been wandering for some time and found nothing.
   */
  if (m_state.time_exploring_unsuccessfully >
      mc_params->times.max_unsuccessful_explore) {
    if (m_rng->Uniform(m_prob_range) < m_state.explore_to_rest_prob) {
      internal_event(ST_EXPLORE_FAIL);
    }
    /* Apply the food rule, increasing explore_to_rest_prob and
     * decreasing RestToExploreProb */
    m_state.explore_to_rest_prob += mc_params->deltas.food_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.rest_to_explore_prob -= mc_params->deltas.food_rule_rest_to_explore;
    m_prob_range.TruncValue(m_state.rest_to_explore_prob);
  }

  /* perform the actual exploration */
  ++m_state.time_exploring_unsuccessfully;

  /* Get the diffusion vector to perform obstacle avoidance */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE, new struct collision_event_data);
  }
  argos::CVector2 diff_vector;
  m_sensors->calc_diffusion_vector(&diff_vector);

  /* No obstacles nearby--use the diffusion vector only to set speeds */
  m_actuators->set_wheel_speeds(m_actuators->max_wheel_speed() * diff_vector);
}
STATE_DEFINE(social_fsm, explore_success, fsm::no_event_data) {
  ER_DIAG("Executing ST_EXPLORE_SUCCESS");

  m_actuators->leds_set_color(argos::CColor::GREEN);

  /*
   * Apply the food rule, decreasing explore_to_rest_prob and increasing
   * RestToExploreProb
   */
  m_state.explore_to_rest_prob -= mc_params->deltas.food_rule_explore_to_rest;
  m_prob_range.TruncValue(m_state.explore_to_rest_prob);
  m_state.rest_to_explore_prob += mc_params->deltas.food_rule_rest_to_explore;
  m_prob_range.TruncValue(m_state.rest_to_explore_prob);

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_SUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}
STATE_DEFINE(social_fsm, explore_fail, fsm::no_event_data) {
  ER_DIAG("Executing ST_EXPLORE_FAIL");
  m_actuators->leds_set_color(argos::CColor::ORANGE);

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_UNSUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
}
STATE_DEFINE(social_fsm, return_to_nest, fsm::no_event_data) {
    ER_DIAG("Executing ST_RETURN_TO_NEST");
    argos::CVector2 vector;

    if (m_sensors->in_nest()) {
      internal_event(ST_SEARCH_FOR_SPOT_IN_NEST);
    }

    m_sensors->calc_diffusion_vector(&vector);
    m_actuators->set_wheel_speeds(
        m_actuators->max_wheel_speed() * vector +
        m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
}
STATE_DEFINE(social_fsm, collision_avoidance, struct collision_event_data) {
  argos::CVector2 vector;

  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");

  while (m_sensors->calc_diffusion_vector(&vector)) {
    vector = -vector.Normalize();

    /* Use the diffusion vector only */
    m_actuators->set_wheel_speeds(vector);
    /*
     * Collision avoidance happened, increase explore_to_rest_prob and decrease
     * RestToExploreProb
     */
    m_state.explore_to_rest_prob += mc_params->deltas.collision_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.rest_to_explore_prob -= mc_params->deltas.collision_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.rest_to_explore_prob);
  } /* while() */

  internal_event(data->last_state);
}
STATE_DEFINE(social_fsm, search_for_spot_in_nest, fsm::no_event_data) {
  ER_DIAG("Executing ST_SEARCH_FOR_SPOT_IN_NEST");

  /* Have we looked for a place long enough? */
  if (m_state.time_search_for_place_in_nest > mc_params->times.min_search_for_place_in_nest) {
    m_actuators->stop_wheels();
    m_actuators->set_raba_data(m_last_explore_res);
    internal_event(ST_REST);
  }

  /* No, keep looking */
  ++m_state.time_search_for_place_in_nest;

  argos::CVector2 vector;
  m_sensors->calc_diffusion_vector(&vector);

  m_actuators->set_wheel_speeds(
      m_actuators->max_wheel_speed() * vector +
      m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
}

ENTRY_DEFINE(social_fsm, entry_rest, fsm::no_event_data) {
  ER_DIAG("Entrying ST_REST");
  m_actuators->leds_set_color(argos::CColor::BLACK);
}
ENTRY_DEFINE(social_fsm, entry_explore, fsm::no_event_data) {
  ER_DIAG("Entrying ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
}
ENTRY_DEFINE(social_fsm, entry_collision_avoidance, fsm::no_event_data) {
  ER_DIAG("Entering ST_COLLIISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}
ENTRY_DEFINE(social_fsm, entry_search_for_spot_in_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_SEARCH_FOR_SPOT_IN_NEST");
  m_actuators->leds_set_color(argos::CColor::YELLOW);
}

EXIT_DEFINE(social_fsm, exit_search_for_spot_in_nest) {
  ER_DIAG("Exiting ST_SEARCH_FOR_SPOT_IN_NEST");
  m_last_explore_res = LAST_EXPLORATION_NONE;
  m_state.time_search_for_place_in_nest = 0;
}
EXIT_DEFINE(social_fsm, exit_explore) {
  ER_DIAG("Exiting ST_EXLORE");
  m_state.time_exploring_unsuccessfully = 0;
  m_state.time_search_for_place_in_nest = 0;
}
EXIT_DEFINE(social_fsm, exit_rest) {
  ER_DIAG("Exiting ST_REST");

  m_state.time_rested = 0;
}


/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void social_fsm::reset(void) {
  m_state.explore_to_rest_prob = mc_params->initial_rest_to_explore_prob;
  m_state.time_exploring_unsuccessfully = 0;

  /*
   * Initially the robot is resting, and by setting RestingTime to
   * MinimumRestingTime we force the robots to make a decision at the experiment
   * start. If instead we set RestingTime to zero, we would have to wait till
   * RestingTime reaches MinimumRestingTime before something happens, which is
   * just a waste of time.
   */
  m_state.time_rested = mc_params->times.min_rested;
  m_state.time_search_for_place_in_nest = 0;
  m_last_explore_res = LAST_EXPLORATION_NONE;
  m_actuators->reset(LAST_EXPLORATION_NONE);
  base_fsm::reset();
} /* reset() */

NS_END(fordyca);
