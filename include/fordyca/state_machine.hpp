/**
 * @file state_machine.hpp
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

#ifndef INCLUDE_FORDYCA_STATE_MACHINE_HPP_
#define INCLUDE_FORDYCA_STATE_MACHINE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * Contains all the state information about the controller.
 */
class state_machine {
 public:
  state_machine();

  /* The three possible states in which the controller can be */
  enum state {
    STATE_RESTING = 0,
    STATE_EXPLORING,
    STATE_RETURN_TO_NEST
  };

  /*
   * Updates the state information.
   * In pratice, it sets the SStateData::InNest flag.
   * Future, more complex implementations should add their
   * state update code here.
   */
  void UpdateState();

  /*
   * Returns true if the robot is currently exploring.
   */
  inline bool is_exploring() const {
    return curr_state_ == STATE_EXPLORING;
  }
  /*
   * Returns true if the robot is currently resting.
   */
  inline bool IsResting() const {
    return curr_state_ == STATE_RESTING;
  }

  /*
   * Returns true if the robot is currently returning to the nest.
   */
  inline bool is_returning_to_nest() const {
    return curr_state_ == STATE_RETURN_TO_NEST;
  }

 private:

  /* True when the robot is in the nest */
  bool InNest;
  struct state_machine_params params;
  enum state curr_state_;
};

/*******************************************************************************
 * Operater Definitions
 ******************************************************************************/

} /* namespace fordyca */

#endif /* INCLUDE_FORDYCA_STATE_MACHINE_HPP_ */
