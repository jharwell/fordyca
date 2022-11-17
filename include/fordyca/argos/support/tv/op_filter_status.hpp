/**
 * \file op_filter_status.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Contains the various statuses relating to robots and block operations
 * (picking up, dropping).
 */
enum class op_filter_status {
  /**
   * \brief The robot has not currently achieved the necessary internal state
   * for the operation.
   */
  ekROBOT_INTERNAL_UNREADY,

  /**
   * \brief The robot has passed all necessary filter checkes for the requested
   * operation.
   */
  ekSATISFIED,

  /**
   * \brief The robot has achieved the necessary internal state for the block
   * operation, but is not actually on a block, so the desired operation is
   * invalid.
   */
  ekROBOT_NOT_ON_BLOCK,

  /**
   * \brief The robot has requested an action that while too close to an
   * existing block in the arena.
   */
  ekBLOCK_PROXIMITY,

  /**
   * \brief The robot has requested an action while too close to an existing
   * cache in the arena.
   */
  ekCACHE_PROXIMITY
};

NS_END(tv, argos support, argos, fordyca);

