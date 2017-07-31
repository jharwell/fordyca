/**
 * @file params.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
n *
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

#ifndef INCLUDE_FORDYCA_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/angles.h>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct base_params {};

struct prob_deltas  {
  /* The increase of explore_to_rest_prob due to the food rule */
  argos::Real food_rule_explore_to_rest;

  /* The increase of curr_rest_to_explore_prob due to the food rule */
  argos::Real food_rule_rest_to_explore;

  /* The increase of explore_to_rest_prob due to the collision rule */
  argos::Real collision_rule_explore_to_rest;

  /* The increase of curr_rest_to_explore_prob due to the social rule */
  argos::Real social_rule_rest_to_explore;

  /* The increase of explore_to_rest_prob due to the social rule */
  argos::Real social_rule_explore_to_rest;
};

struct threshold_times {
  /* The minimum number of steps in resting state before the robots
     starts thinking that it's time to move */
  size_t min_rested;

  /* The number of exploration steps without finding food after which
     a foot-bot starts thinking about going back to the nest */
  size_t max_unsuccessful_explore;
  /*
   * If the robots switched to resting as soon as it enters the nest, there
   * would be overcrowding of robots in the border between the nest and the
   * rest of the arena. To overcome this issue, the robot spends some time
   * looking for a place in the nest before finally settling. The following
   * variable contains the minimum time the robot must spend in state 'return
   * to nest' looking for a place in the nest before switching to the resting
   * state.
   */
  size_t min_search_for_place_in_nest;
};

struct social_fsm_params : public base_params {
  /* Initial probability to switch from resting to exploring */
  argos::Real initial_rest_to_explore_prob;

  /* Initial probability to switch from exploring to resting */
  argos::Real initial_explore_to_rest_prob;

  struct prob_deltas deltas;
  struct threshold_times times;
};

/*
 * The following variables are used as parameters for turning during
 * navigation. You can set their value in the <parameters> section of the XML
 * configuration file, under the
 * <controllers><footbot_foraging_controller><parameters><wheel_turning>
 * section.
 */
struct wheel_params {
  /* Angular thresholds to change turning state. */
  argos::CRadians hard_turn_threshold;
  argos::CRadians soft_turn_threshold;
  argos::CRadians no_turn_threshold;
  argos::Real max_speed;

  wheel_params(void) :
      hard_turn_threshold(),
      soft_turn_threshold(),
      no_turn_threshold(),
      max_speed() {}
};

/*
 * The following variables are used as parameters for the diffusion
 * algorithm. You can set their value in the <parameters> section of the XML
 * configuration file, under the
 * <controllers><footbot_foraging_controller><parameters><diffusion> section.
x */
struct diffusion_params {
  diffusion_params() :
      delta(0.0),
      go_straight_angle_range(argos::CRadians(-1.0f), argos::CRadians(1.0f)) {}
  /*
   * Maximum tolerance for the proximity reading between
   * the robot and the closest obstacle.
   * The proximity reading is 0 when nothing is detected
   * and grows exponentially to 1 when the obstacle is
   * touching the robot.
   */
  argos::Real delta;
  /* Angle tolerance range to go straight. */
  argos::CRange<argos::CRadians> go_straight_angle_range;

};

struct actuator_params : public base_params {
  actuator_params(void) : wheels() {}

  struct wheel_params wheels;

};

struct sensor_params : public base_params {
  sensor_params(void) : diffusion() {}

  struct diffusion_params diffusion;
};

struct food_params : public base_params {
  uint n_items;
  argos::Real square_radius;
  uint energy_per_item;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_HPP_ */
