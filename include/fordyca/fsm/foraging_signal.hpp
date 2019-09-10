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

#ifndef INCLUDE_FORDYCA_FSM_FORAGING_SIGNAL_HPP_
#define INCLUDE_FORDYCA_FSM_FORAGING_SIGNAL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

#include "cosm/fsm/util_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class foraging_signal
 * @ingroup fordyca fsm
 *
 * @brief Signals that FSMs can use to communicate between sub/super states, and
 * that can be used to direct them in some way.
 */
class foraging_signal : public cfsm::util_signal {
 public:
  enum type {
    /**
     * @brief A robot has picked up a block
     */
    ekBLOCK_PICKUP = cfsm::util_signal::type::ekEXTERNAL_SIGNALS,

    /**
     * @brief A robot has dropped a block in {cache, nest, arena}
     */
    ekBLOCK_DROP,

    /**
     * @brief Direct a robot to acquire a free block in the arena
     */
    ekACQUIRE_FREE_BLOCK,

    /**
     * @brief Direct a robot to acquire a block from a cache.
     */
    ekACQUIRE_CACHED_BLOCK,

    /**
     * @brief The block a robot was waiting to pickup from has vanished (see
     * #411).
     */
    ekBLOCK_VANISHED,

    /**
     * @brief The proximity of a block unknown to the robot is blocnking it from
     * completing its current task. Used by the Cache Starter task.
     */
    ekBLOCK_PROXIMITY,

    /**
     * @brief The cache the robot was waiting to pickup from has vanished (see
     * #247).
     */
    ekCACHE_VANISHED,
    /**
     * @brief The place the robot wait waiting to drop a block into has suddenly
     * become a cache.
    */
    ekCACHE_PROXIMITY
  };
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_FORAGING_SIGNAL_HPP_ */
