/**
 * \file loop_utils.hpp
 * \ingroup support utils
 *
 * Helpers for loop functions that CAN be free functions, as they do not require
 * access to anything in \ref argos::CLoopFunctions.
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/vector2.hpp"
#include "fordyca/fordyca.hpp"
#include "cosm/ds/block2D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

NS_START(fordyca, support, utils);

/******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * \brief Calculate the blocks that are:
 *
 * - Not carried by a robot
 * - Not inside a cache
 *
 * \param all_caches All existing caches in the arena.
 * \param all_blocks All blocks in the arena.
 */
cds::block2D_vectorno free_blocks_calc(const cads::acache_vectoro& all_caches,
                                       const cds::block2D_vectorno& all_blocks);

NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_ */
