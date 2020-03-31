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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCKS_BLOCK_MANIP_EVENTS_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCKS_BLOCK_MANIP_EVENTS_HPP_

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
enum block_manip_events {
  ekFREE_PICKUP,
  ekFREE_DROP,
  ekCACHE_PICKUP,
  ekCACHE_DROP,
  ekMAX_EVENTS
};

NS_END(blocks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCKS_BLOCK_MANIP_EVENTS_HPP_ */
