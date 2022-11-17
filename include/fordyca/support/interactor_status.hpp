/**
 * \file interactor_status.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The status returned by an action class representing a robot
 * interaction with the arena/environment. Used to disambiguate when
 * post-interaction hooks need to be run after processing all interactions for a
 * given robot on a given timestep.
 */
enum class interactor_status {
  /**
   * \brief No event occured (i.e. the robot did not meet the requirements for
   * the interaction).
   */
  ekNO_EVENT = 1 << 0,

  /**
   * \brief The robot dropped a block in the nest, from \ref nest_interactor.
   */
  ekNEST_BLOCK_PROCESS = 1 << 1,

  /**
   * \brief The robot picked up a free block in the arena, from \ref
   * free_block_interactor.
   */
  ekARENA_FREE_BLOCK_PICKUP = 1 << 2,

  /**
   * \brief The robot purposefully dropped a block in the arena, NOT as a part
   * of a task abort, to start a new cache.
   */
  ekNEW_CACHE_BLOCK_DROP = 1 << 3,

  /**
   * \brief The robot aborted their current task, possibly resulting in a free
   * block drop, from \ref task_abort_interactor.
   */
  ekTASK_ABORT = 1 << 4,

  /**
   * \brief A cache has been depleted as a result of a robot picking up the last
   * block from it.
   */
  ekCACHE_DEPLETION = 1 << 5,

  /**
   * \brief The robot purposefully dropped a block somewhere in the arena to
   * finish a new cache, from \ref new_cache_block_drop_interactor.
   */
  ekFREE_BLOCK_DROP = 1 << 6,
};

NS_END(support, fordyca);
