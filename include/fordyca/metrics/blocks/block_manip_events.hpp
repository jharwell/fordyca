/**
 * \file block_manip_events.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \enum The types of ways that blocks can be manipulated.
 */
enum block_manip_events {
  /**
   * \brief A block been picked up in the arena outside of a cache.
   */
  ekFREE_PICKUP,

  /**
   * \brief A block been dropped in the arena outside of a cache.
   */
  ekFREE_DROP,

  /**
   * \brief A block been picked up from a cache.
   */
  ekCACHE_PICKUP,

  /**
   * \brief A block been dropped into a cache.
   */
  ekCACHE_DROP,
  ekMAX_EVENTS
};

NS_END(blocks, metrics, fordyca);

