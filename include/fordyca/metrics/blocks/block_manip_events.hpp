/**
 * \file block_manip_events.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

