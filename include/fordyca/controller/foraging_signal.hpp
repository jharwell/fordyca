/**
 * @file foraging_signal.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_FORAGING_SIGNAL_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_FORAGING_SIGNAL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/state_machine/event.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class foraging_signal
 * @ingroup controller
 *
 * @brief Signals that sub-states can return in order to notify their super
 * states that a condition that they do not know how to handle has arisen.
 */
class foraging_signal : public state_machine::event_signal {
 public:
  enum type {
    /**
     * The signal sent to FSMs during nominal operation.
     */
    FSM_RUN = state_machine::event_signal::EXTERNAL_SIGNALS,
    BLOCK_PICKUP,       /// A robot has picked up a block
    BLOCK_DROP,         /// A robot has dropped a block in {cache, nest, arena}
    LEFT_NEST,          /// A robot has left the nest
    ACQUIRE_FREE_BLOCK, /// Direct a robot to acquire a free block in the arena
    ACQUIRE_CACHED_BLOCK, /// Direct a robot to acquire a block from a cache.
    COLLISION_IMMINENT,    /// The robot is about to collide!
    /**
     * @brief The cache the robot was waiting to pickup from has vanished (see
     * #247).
     */
    CACHE_VANISHED
  };
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_SIGNAL_HPP_ */
