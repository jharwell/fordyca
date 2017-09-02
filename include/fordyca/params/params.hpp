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

#ifndef INCLUDE_FORDYCA_PARAMS_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector2.h>
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
};

struct threshold_times {
  /*
   * The number of exploration steps without finding block after which the
   * footboot will randomly change direction.
   */
  size_t unsuccessful_explore_dir_change;
  /*
   * The number of time steps between two successive collisions that will be
   * considered excessive, and result in a random direction being added to the
   * avoidance heading to help avoid collisions in the immediate future.
   */
  uint frequent_collision_thresh;
};

struct foraging_fsm_params : public base_params {
  foraging_fsm_params(void) : deltas(), times(), nest_center() {}
  struct prob_deltas deltas;
  struct threshold_times times;
  argos::CVector2 nest_center;
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
 * algorithm.
 */
struct diffusion_params {
  diffusion_params() :
      delta(0.0),
      go_straight_angle_range(argos::CRadians(-1.0f), argos::CRadians(1.0f)) {}
  /*
   * Maximum tolerance for the proximity reading between the robot and the
   * closest obstacle.  The proximity reading is 0 when nothing is detected and
   * grows exponentially to 1 when the obstacle is touching the robot.
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

struct block_params : public base_params {
  block_params(uint n_blocks_ = 0, argos::Real dimension_ = 0.0,
               std::string dist_model_ =  "", bool respawn_ = false) :
      n_blocks(n_blocks_), dimension(dimension_), dist_model(dist_model_),
      respawn(respawn_) {}

  uint n_blocks;
  argos::Real dimension;
  std::string dist_model;
  bool respawn;
};

struct logging_params : public base_params {
  logging_params(void) : sim_stats() {}

  std::string sim_stats;
};

struct loop_functions_params : public base_params {
  loop_functions_params(void) :
      nest_x(), nest_y(), display_robot_id(false), display_robot_los(false),
      display_block_id(false) {}
  argos::CRange<argos::Real> nest_x;
  argos::CRange<argos::Real> nest_y;
  bool display_robot_id;
  bool display_robot_los;
  bool display_block_id;
};

struct grid_params : public base_params {
  grid_params(double resolution_ = 0.0,
              argos::CVector2 upper_ = argos::CVector2(),
              argos::CVector2 lower_ = argos::CVector2(),
              struct block_params block_ = block_params()) :
      resolution(resolution_), upper(upper_), lower(lower_), block(block_) {}
  double resolution;
  argos::CVector2 upper;
  argos::CVector2 lower;
  struct block_params block;
};

struct perceived_grid_params : public base_params {
  perceived_grid_params(void) : grid(), pheromone_rho(0.0) {}
  struct grid_params grid;
  double pheromone_rho;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_PARAMS_HPP_ */
