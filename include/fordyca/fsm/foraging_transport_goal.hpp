/**
 * \file foraging_transport_goal.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief Represents the different locations/entities that a robot can transport
 * a block to during foraging once they have picked one up.
 */
enum class foraging_transport_goal {
  /**
   * \brief No goal--robot is probably not carrying a block.
   */
  ekNONE = -1,

  /**
   * \brief A robot has acquired a block and is currently taking it back to
   * the nest.
   */
  ekNEST,

  /**
   * \brief A robot is currently transporting an acquired block to its
   * existing cache of choice.
   */
  ekEXISTING_CACHE,

  /**
   * \brief A robot is currently transporting an acquired block to its new
   * cache of choice.
   */
  ekNEW_CACHE,

  /**
   * \brief A robot is currently transporting an acquired block to its cache
   * site of choice.
   */
  ekCACHE_SITE
};

NS_END(fsm, fordyca);
