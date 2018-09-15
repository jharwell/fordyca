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
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/throttling_differential_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;
namespace utils = rcppsw::utils;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vector_fsm::vector_fsm(controller::saa_subsystem* const saa)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.vector"),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      entry_new_direction(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(vector, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_recovery, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(arrived, hfsm::top_state()),
      entry_vector(),
      entry_collision_avoidance(),
      entry_collision_recovery(),
      m_state(),
      m_goal_data() {}

/*******************************************************************************
 * States
 ******************************************************************************/
__rcsw_const FSM_STATE_DEFINE_ND(vector_fsm, start) {
  return controller::foraging_signal::HANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_avoidance) {
  if (ST_COLLISION_AVOIDANCE != last_state()) {
    ER_DEBUG("Executing ST_COLLIISION_AVOIDANCE");
  }
  /*
   * If we came from the NEW_DIRECTION_STATE, then we got there from this
   * function, and have just finished changing our direction due to a frequent
   * collision. As such, we need to go into collision recovery, and zoom in our
   * new direction away from whatever is causing the problem. See #243.
   */
  if (ST_NEW_DIRECTION == previous_state()) {
    actuators()->differential_drive().set_wheel_speeds(
        actuators()->differential_drive().max_speed() * 0.7,
        actuators()->differential_drive().max_speed() * 0.7);
    collision_avoidance_tracking_end();
    internal_event(ST_COLLISION_RECOVERY);
    return controller::foraging_signal::HANDLED;
  }

  if (base_sensors()->threatening_obstacle_exists()) {
    collision_avoidance_tracking_begin();
    if (base_sensors()->tick() - m_state.last_collision_time <
        kFREQ_COLLISION_THRESH) {
      ER_DEBUG("Frequent collision: last=%u curr=%u",
               m_state.last_collision_time,
               base_sensors()->tick());
      argos::CVector2 new_dir = randomize_vector_angle(argos::CVector2::X);
      internal_event(ST_NEW_DIRECTION,
                     rcppsw::make_unique<new_direction_data>(new_dir.Angle()));
    } else {
      argos::CVector2 obs = base_sensors()->find_closest_obstacle();
      ER_DEBUG("Found threatening obstacle: (%f, %f)@%f [%f]",
               obs.GetX(),
               obs.GetY(),
               obs.Angle().GetValue(),
               obs.Length());
      saa_subsystem()->steering_force().avoidance(obs);
      /*
       * If we are currently spinning in place (hard turn), we have 0 linear
       * velocity, and that does not play well with the arrival force
       * calculations. To fix this, and a bit of wander force.
       */
      if (saa_subsystem()->linear_velocity().Length() <= 0.1) {
        saa_subsystem()->steering_force().wander();
      }
      saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    }
  } else {
    m_state.last_collision_time = base_sensors()->tick();
    /*
     * Go in whatever direction you are currently facing for collision recovery.
     */
    actuators()->differential_drive().set_wheel_speeds(
        actuators()->differential_drive().max_speed() * 0.7,
        actuators()->differential_drive().max_speed() * 0.7);
    collision_avoidance_tracking_end();
    internal_event(ST_COLLISION_RECOVERY);
  }
  return controller::foraging_signal::HANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_recovery) {
  if (ST_COLLISION_RECOVERY != last_state()) {
    ER_DEBUG("Executing ST_COLLISION_RECOVERY");
  }

  if (++m_state.m_collision_rec_count >= kCOLLISION_RECOVERY_TIME) {
    m_state.m_collision_rec_count = 0;
    internal_event(ST_VECTOR);
  }
  return controller::foraging_signal::HANDLED;
}
FSM_STATE_DEFINE(vector_fsm, vector, state_machine::event_data) {
  if (ST_VECTOR != last_state()) {
    ER_DEBUG("Executing ST_VECTOR");
  }

  auto* goal = dynamic_cast<const struct goal_data*>(data);
  if (nullptr != goal) {
    m_goal_data = *goal;
    ER_INFO("Target: (%f, %f)", m_goal_data.loc.GetX(), m_goal_data.loc.GetY());
  }

  if ((m_goal_data.loc - base_sensors()->position()).Length() <=
      m_goal_data.tolerance) {
    internal_event(ST_ARRIVED,
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
  if (base_sensors()->threatening_obstacle_exists() &&
      !saa_subsystem()->steering_force().within_slowing_radius()) {
    internal_event(ST_COLLISION_AVOIDANCE);
  } else {
    saa_subsystem()->steering_force().seek_to(m_goal_data.loc);
    saa_subsystem()->actuation()->leds_set_color(utils::color::kBLUE);
    saa_subsystem()->apply_steering_force(std::make_pair(true, false));
  }
  return controller::foraging_signal::HANDLED;
}

FSM_STATE_DEFINE(vector_fsm, arrived, struct goal_data) {
  if (ST_ARRIVED != last_state()) {
    ER_DEBUG("Executing ST_ARRIVED: target (%f, %f) within %f tolerance",
             data->loc.GetX(),
             data->loc.GetY(),
             data->tolerance);
  }
  return controller::foraging_signal::HANDLED;
}

FSM_ENTRY_DEFINE_ND(vector_fsm, entry_vector) {
  ER_DEBUG("Entering ST_VECTOR");
  actuators()->leds_set_color(utils::color::kBLUE);
}
FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_avoidance) {
  ER_DEBUG("Entering ST_COLLISION_AVOIDANCE");
  actuators()->leds_set_color(utils::color::kRED);
}
FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_recovery) {
  ER_DEBUG("Entering ST_COLLISION_RECOVERY");
  actuators()->leds_set_color(utils::color::kYELLOW);
}
/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool vector_fsm::in_collision_avoidance(void) const {
  return ST_COLLISION_AVOIDANCE == current_state();
} /* in_collision_avoidance() */

bool vector_fsm::entered_collision_avoidance(void) const {
  return ST_COLLISION_AVOIDANCE != last_state() && in_collision_avoidance();
} /* entered_collision_avoidance() */

bool vector_fsm::exited_collision_avoidance(void) const {
  return ST_COLLISION_AVOIDANCE == last_state() && !in_collision_avoidance();
} /* exited_collision_avoidance() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void vector_fsm::task_start(
    const rcppsw::task_allocation::taskable_argument* const c_arg) {
  static const uint8_t kTRANSITIONS[] = {
      ST_VECTOR,                            /* start */
      ST_VECTOR,                            /* vector */
      controller::foraging_signal::IGNORED, /* collision avoidance */
      controller::foraging_signal::IGNORED, /* collision recovery */
      controller::foraging_signal::IGNORED, /* new direction */
      controller::foraging_signal::IGNORED, /* arrived */
  };
  auto* const a = dynamic_cast<const tasks::vector_argument*>(c_arg);
  ER_ASSERT(nullptr != a, "bad argument passed");
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<struct goal_data>(a->vector(),
                                                       a->tolerance()));
} /* task_start() */

void vector_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

void vector_fsm::init(void) {
  actuators()->reset();
  state_machine::simple_fsm::init();
} /* init() */

__rcsw_pure argos::CVector2 vector_fsm::calc_vector_to_goal(
    const argos::CVector2& goal) {
  return goal - base_sensors()->position();
} /* calc_vector_to_goal() */

NS_END(fsm, fordyca);
