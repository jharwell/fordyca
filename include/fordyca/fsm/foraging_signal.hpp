/**
 * \file foraging_signal.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/fsm/util_signal.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_signal
 * \ingroup fsm
 *
 * \brief Signals that FSMs can use to communicate between sub/super states, and
 * that can be used to direct them in some way.
 */
class foraging_signal : public csfsm::util_signal {
 public:
  enum type {
    /**
     * \brief A robot has picked up a block
     */
    ekBLOCK_PICKUP = csfsm::util_signal::type::ekEXTERNAL_SIGNALS,

    /**
     * \brief A robot has dropped a block in {cache, nest, arena}
     */
    ekBLOCK_DROP,

    /**
     * \brief Direct a robot to acquire a free block in the arena
     */
    ekACQUIRE_FREE_BLOCK,

    /**
     * \brief Direct a robot to acquire a block from a cache.
     */
    ekACQUIRE_CACHED_BLOCK,

    /**
     * \brief The block a robot was waiting to pickup from has vanished (see
     * FORDYCA#411).
     */
    ekBLOCK_VANISHED,

    /**
     * \brief The proximity of a block unknown to the robot is blocnking it from
     * completing its current task. Used by the Cache Starter task.
     */
    ekBLOCK_PROXIMITY,

    /**
     * \brief The cache the robot was waiting to pickup from has vanished (see
     * FORDYCA#247).
     */
    ekCACHE_VANISHED,
    /**
     * \brief The place the robot wait waiting to drop a block into has suddenly
     * become a cache.
    */
    ekCACHE_PROXIMITY
  };
};

NS_END(controller, fordyca);
