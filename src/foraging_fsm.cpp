/**
 * @file foraging_fsm.cpp
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
#include "fordyca/foraging_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
foraging_fsm::foraging_fsm(const struct foraging_fsm_params* params,
                       std::shared_ptr<rcppsw::common::er_server> server,
                       std::shared_ptr<sensor_manager> sensors,
                       std::shared_ptr<actuator_manager> actuators) :
    fsm::simple_fsm(server, ST_MAX_STATES),
    rest(),
    explore(),
    explore_success(),
    explore_fail(),
    return_to_nest(),
    leaving_nest(),
    search_for_spot_in_nest(),
    collision_avoidance(),
    exit_search_for_spot_in_nest(),
    entry_explore(),
    entry_rest(),
    entry_collision_avoidance(),
    entry_search_for_spot_in_nest(),
    entry_leaving_nest(),
    m_last_explore_res(LAST_EXPLORATION_NONE),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_prob_range(0.0f, 1.0f),
    m_state(),
    mc_params(params),
    m_sensors(sensors),
    m_actuators(actuators) {
  insmod("foraging_fsm");
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::NOM);
    }

/*******************************************************************************
 * Events
 ******************************************************************************/
void foraging_fsm::event_explore(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    ST_LEAVING_NEST,            /* resting */
    fsm::event::EVENT_IGNORED,  /* explore */
    fsm::event::EVENT_IGNORED,  /* explore success */
    fsm::event::EVENT_IGNORED,  /* explore fail */
    fsm::event::EVENT_IGNORED,  /* return to nest */
    fsm::event::EVENT_IGNORED,  /* leaving nest */
    fsm::event::EVENT_IGNORED,  /* search for spot in nest */
    ST_COLLISION_AVOIDANCE,     /* collision avoidance */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_block_found(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    fsm::event::EVENT_FATAL,     /* resting */
    ST_EXPLORE_SUCCESS,          /* explore */
    fsm::event::EVENT_IGNORED,   /* explore success */
    fsm::event::EVENT_IGNORED,   /* explore fail */
    fsm::event::EVENT_IGNORED,   /* return to nest */
    fsm::event::EVENT_FATAL,     /* leaving to nest */
    fsm::event::EVENT_IGNORED,   /* search for spot in nest */
    ST_COLLISION_AVOIDANCE,      /* collision avoidance */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_continue(void) {
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
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_entered_nest(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    fsm::event::EVENT_FATAL,         /* resting */
        fsm::event::EVENT_IGNORED,   /* explore */
        fsm::event::EVENT_FATAL,     /* explore success */
        fsm::event::EVENT_FATAL,     /* explore fail */
        ST_SEARCH_FOR_SPOT_IN_NEST,  /* return to nest */
        fsm::event::EVENT_FATAL,     /* leaving nest */
        fsm::event::EVENT_IGNORED,   /* search for spot in nest */
        ST_COLLISION_AVOIDANCE,     /* collision avoidance */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(foraging_fsm, rest, fsm::no_event) {
  ER_DIAG("Executing ST_REST rested=%zu min=%zu rest_to_explore=%f",
          m_state.time_rested, mc_params->times.min_rested,
          m_state.rest_to_explore_prob);

  /*
   * If we have stayed here enough, probabilistically switch to 'exploring'
   */
  if (m_state.time_rested > mc_params->times.min_rested &&
       m_rng->Uniform(m_prob_range) > m_state.rest_to_explore_prob) {
    internal_event(ST_LEAVING_NEST);
  }

  /* continue resting */
  ++m_state.time_rested;
  /* Be sure not to send the last exploration result multiple times */
  if (m_state.time_rested == 1) {
    m_actuators->set_raba_data(LAST_EXPLORATION_NONE);
  }
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, leaving_nest, fsm::no_event) {
  ER_DIAG("Executing ST_LEAVING_NEST");

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
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE");

  /*
   * We transition to the 'return to nest' state in two situations:
   *
   * 1. If we have a block item, in which case we are notified by an external event.
   * 2. If we have not found a block item for some time; in this case, the switch
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
    /* Apply the block rule, increasing explore_to_rest_prob and
     * decreasing RestToExploreProb */
    m_state.explore_to_rest_prob += mc_params->deltas.block_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.rest_to_explore_prob -= mc_params->deltas.block_rule_rest_to_explore;
    m_prob_range.TruncValue(m_state.rest_to_explore_prob);
  }

  /* perform the actual exploration */
  ++m_state.time_exploring_unsuccessfully;

  /* Get the diffusion vector to perform obstacle avoidance */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  }
  argos::CVector2 diff_vector;
  m_sensors->calc_diffusion_vector(&diff_vector);

  /* No obstacles nearby--use the diffusion vector only to set speeds */
  m_actuators->set_wheel_speeds(m_actuators->max_wheel_speed() * diff_vector);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore_success, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE_SUCCESS");

  /*
   * Apply the block rule, decreasing explore_to_rest_prob and increasing
   * RestToExploreProb
   */
  m_state.explore_to_rest_prob -= mc_params->deltas.block_rule_explore_to_rest;
  m_prob_range.TruncValue(m_state.explore_to_rest_prob);
  m_state.rest_to_explore_prob += mc_params->deltas.block_rule_rest_to_explore;
  m_prob_range.TruncValue(m_state.rest_to_explore_prob);

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_SUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore_fail, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE_FAIL");

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_UNSUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, return_to_nest, fsm::no_event) {
    ER_DIAG("Executing ST_RETURN_TO_NEST");
    argos::CVector2 vector;

    if (LAST_EXPLORATION_SUCCESSFUL == m_last_explore_res) {
      m_actuators->leds_set_color(argos::CColor::GREEN);
    } else {
      m_actuators->leds_set_color(argos::CColor::ORANGE);
    }

    if (m_sensors->in_nest()) {
      internal_event(ST_SEARCH_FOR_SPOT_IN_NEST);
    }

    m_sensors->calc_diffusion_vector(&vector);
    /*   internal_event(ST_COLLISION_AVOIDANCE); */
    /* } */
      m_actuators->set_wheel_speeds(
        m_actuators->max_wheel_speed() * vector +
        m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
      return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, collision_avoidance, fsm::no_event) {
  argos::CVector2 vector;
  static argos::CColor last_color;
  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");
  if (m_sensors->calc_diffusion_vector(&vector)) {

    m_actuators->set_wheel_speeds(vector);
    /*
     * Collision avoidance happened, increase explore_to_rest_prob and decrease
     * RestToExploreProb
     */
    m_state.explore_to_rest_prob += mc_params->deltas.collision_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.explore_to_rest_prob);
    m_state.rest_to_explore_prob -= mc_params->deltas.collision_rule_explore_to_rest;
    m_prob_range.TruncValue(m_state.rest_to_explore_prob);
  } else {
    internal_event(previous_state());
  }
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, search_for_spot_in_nest, fsm::no_event) {
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
  /* if (m_sensors->calc_diffusion_vector(&vector)) { */
  /*   internal_event(ST_COLLISION_AVOIDANCE); */
  /* } */
  m_sensors->calc_diffusion_vector(&vector);

  m_actuators->set_wheel_speeds(
      m_actuators->max_wheel_speed() * vector +
      m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
  return fsm::event::EVENT_HANDLED;
}

FSM_ENTRY_DEFINE(foraging_fsm, entry_rest, fsm::no_event) {
  ER_DIAG("Entrying ST_REST");
  m_actuators->leds_set_color(argos::CColor::BLACK);
  m_actuators->stop_wheels();
  m_state.time_rested = 0;
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_leaving_nest, fsm::no_event) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_explore, fsm::no_event) {
  ER_DIAG("Entrying ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
  m_state.time_exploring_unsuccessfully = 0;
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_collision_avoidance, fsm::no_event) {
  ER_DIAG("Entering ST_COLLIISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_search_for_spot_in_nest, fsm::no_event) {
  ER_DIAG("Entering ST_SEARCH_FOR_SPOT_IN_NEST");
  m_actuators->leds_set_color(argos::CColor::YELLOW);
  m_state.time_search_for_place_in_nest = 0;
}

FSM_EXIT_DEFINE(foraging_fsm, exit_search_for_spot_in_nest) {
  ER_DIAG("Exiting ST_SEARCH_FOR_SPOT_IN_NEST");
  m_last_explore_res = LAST_EXPLORATION_NONE;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void foraging_fsm::init(void) {
  m_state.explore_to_rest_prob = mc_params->initial_explore_to_rest_prob;
  m_state.rest_to_explore_prob = mc_params->initial_rest_to_explore_prob;
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
  simple_fsm::init();
} /* init() */

NS_END(fordyca);
