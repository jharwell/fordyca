/**
 * @file vector_fsm.cpp
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
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/controller/throttling_differential_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vector_fsm::vector_fsm(controller::saa_subsystem* const saa)
    : base_foraging_fsm(saa, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.vector"),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(vector, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_recovery, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(arrived, hfsm::top_state()) {}

/*******************************************************************************
 * States
 ******************************************************************************/
__rcsw_const FSM_STATE_DEFINE_ND(vector_fsm, start) {
  return controller::foraging_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_avoidance) {
  if (ekST_COLLISION_AVOIDANCE != last_state()) {
    ER_DEBUG("Executing ekST_COLLIISION_AVOIDANCE");
  }
  /*
   * If we came from the NEW_DIRECTION_STATE, then we got there from this
   * function, and have just finished changing our direction due to a frequent
   * collision. As such, we need to go into collision recovery, and zoom in our
   * new direction away from whatever is causing the problem. See #243.
   */
  if (ekST_NEW_DIRECTION == previous_state()) {
    actuators()->differential_drive().set_wheel_speeds(
        actuators()->differential_drive().max_speed() * 0.7,
        actuators()->differential_drive().max_speed() * 0.7);
    ca_tracker()->ca_enter();
    internal_event(ekST_COLLISION_RECOVERY);
    return controller::foraging_signal::ekHANDLED;
  }

  if (sensors()->threatening_obstacle_exists()) {
    ca_tracker()->ca_exit();
    if (sensors()->tick() - m_state.last_collision_time <
        kFREQ_COLLISION_THRESH) {
      ER_DEBUG("Frequent collision: last=%u curr=%u",
               m_state.last_collision_time,
               sensors()->tick());
      rmath::vector2d new_dir = randomize_vector_angle(rmath::vector2d::X);
      internal_event(ekST_NEW_DIRECTION,
                     rcppsw::make_unique<new_direction_data>(new_dir.angle()));
    } else {
      rmath::vector2d obs = sensors()->find_closest_obstacle();
      ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
               obs.to_str().c_str(),
               obs.angle().value(),
               obs.length());
      saa_subsystem()->steer2D_force_calc().avoidance(obs);
      /*
       * If we are currently spinning in place (hard turn), we have 0 linear
       * velocity, and that does not play well with the arrival force
       * calculations. To fix this, add a bit of wander force.
       */
      if (saa_subsystem()->linear_velocity().length() <= 0.1) {
        saa_subsystem()->steer2D_force_calc().wander();
      }
      saa_subsystem()->steer2D_force_apply(std::make_pair(false, false));
    }
  } else {
    m_state.last_collision_time = sensors()->tick();
    /*
     * Go in whatever direction you are currently facing for collision recovery.
     */
    actuators()->differential_drive().set_wheel_speeds(
        actuators()->differential_drive().max_speed() * 0.7,
        actuators()->differential_drive().max_speed() * 0.7);
    ca_tracker()->ca_exit();
    internal_event(ekST_COLLISION_RECOVERY);
  }
  return controller::foraging_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_recovery) {
  if (ekST_COLLISION_RECOVERY != last_state()) {
    ER_DEBUG("Executing ekST_COLLISION_RECOVERY");
  }

  /*
   * Even though we are recovering from our last collision, and there are
   * hopefully no obstacles nearby, we still need to check if another obstacle
   * threatens. Failure to do this can lead to situations where a robot is
   * "recovering" and moving directly towards a wall, and if the wall is
   * currently just out of range of their proximity sensor upon entering this
   * state, then *occasionally* the robot will end up moving inside of the wall,
   * and somehow the physics engine bounceback does not handle that correctly
   * (or maybe that is how it is supposed to work; idk). This causes an
   * exception and ARGoS crashes. See #519.
   */
  if (sensors()->threatening_obstacle_exists()) {
    m_state.m_collision_rec_count = 0;
    internal_event(ekST_COLLISION_AVOIDANCE);
  } else if (++m_state.m_collision_rec_count >= kCOLLISION_RECOVERY_TIME) {
    m_state.m_collision_rec_count = 0;
    internal_event(ekST_VECTOR);
  }
  return controller::foraging_signal::ekHANDLED;
}
FSM_STATE_DEFINE(vector_fsm, vector, rpfsm::event_data* data) {
  if (ekST_VECTOR != last_state()) {
    ER_DEBUG("Executing ekST_VECTOR");
  }

  auto* goal = dynamic_cast<const struct goal_data*>(data);
  if (nullptr != goal) {
    m_goal_data = *goal;
    ER_INFO("Target=%s, robot=%s",
            m_goal_data.loc.to_str().c_str(),
            saa_subsystem()->sensing()->position().to_str().c_str());
  }

  if ((m_goal_data.loc - sensors()->position()).length() <=
      m_goal_data.tolerance) {
    internal_event(ekST_ARRIVED,
                   rcppsw::make_unique<struct goal_data>(m_goal_data));
  }

  /*
   * Only go into collision avoidance if we are not really close to our
   * target. If we are close, then ignore obstacles (the other guy will
   * move!). 'MURICA.
   *
   * Not doing this results in robots getting stuck when they all are trying to
   * pick up the same block in close quarters.
   */
  if (sensors()->threatening_obstacle_exists() &&
      !saa_subsystem()->steer2D_force_calc().within_slowing_radius()) {
    internal_event(ekST_COLLISION_AVOIDANCE);
  } else {
    saa_subsystem()->steer2D_force_calc().seek_to(m_goal_data.loc);
    saa_subsystem()->actuation()->leds_set_color(rutils::color::kBLUE);
    saa_subsystem()->steer2D_force_apply(std::make_pair(true, false));
  }
  return controller::foraging_signal::ekHANDLED;
}

__rcsw_const FSM_STATE_DEFINE(vector_fsm,
                              arrived,
                              __rcsw_unused struct goal_data* data) {
  if (ekST_ARRIVED != last_state()) {
    ER_DEBUG("Executing ekST_ARRIVED: target=%s, tol=%f",
             data->loc.to_str().c_str(),
             data->tolerance);
  }
  return controller::foraging_signal::ekHANDLED;
}

FSM_ENTRY_DEFINE_ND(vector_fsm, entry_vector) {
  ER_DEBUG("Entering ekST_VECTOR");
  actuators()->leds_set_color(rutils::color::kBLUE);
}
FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_avoidance) {
  ER_DEBUG("Entering ekST_COLLISION_AVOIDANCE");
  actuators()->leds_set_color(rutils::color::kRED);
}
FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_recovery) {
  ER_DEBUG("Entering ekST_COLLISION_RECOVERY");
  actuators()->leds_set_color(rutils::color::kYELLOW);
}
/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
__rcsw_pure bool vector_fsm::in_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE == current_state();
} /* in_collision_avoidance() */

__rcsw_pure bool vector_fsm::entered_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE != last_state() && in_collision_avoidance();
} /* entered_collision_avoidance() */

__rcsw_pure bool vector_fsm::exited_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE == last_state() && !in_collision_avoidance();
} /* exited_collision_avoidance() */

rmath::vector2u vector_fsm::avoidance_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* avoidance_loc() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void vector_fsm::task_start(const rta::taskable_argument* const c_arg) {
  static const uint8_t kTRANSITIONS[] = {
      ekST_VECTOR,                            /* start */
      ekST_VECTOR,                            /* vector */
      controller::foraging_signal::ekIGNORED, /* collision avoidance */
      controller::foraging_signal::ekIGNORED, /* collision recovery */
      controller::foraging_signal::ekIGNORED, /* new direction */
      controller::foraging_signal::ekIGNORED, /* arrived */
  };
  auto* const a = dynamic_cast<const tasks::vector_argument*>(c_arg);
  ER_ASSERT(nullptr != a, "bad argument passed");
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<struct goal_data>(a->vector(),
                                                       a->tolerance()));
} /* task_start() */

void vector_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::ekFSM_RUN,
               rpfsm::event_type::ekNORMAL);
} /* task_execute() */

void vector_fsm::init(void) {
  actuators()->reset();
  rpfsm::simple_fsm::init();
} /* init() */

__rcsw_pure rmath::vector2d vector_fsm::calc_vector_to_goal(
    const rmath::vector2d& goal) {
  return goal - sensors()->position();
} /* calc_vector_to_goal() */

NS_END(fsm, fordyca);
