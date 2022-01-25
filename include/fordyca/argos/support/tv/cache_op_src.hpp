/**
 * \file cache_op_src.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_SRC_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_SRC_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The different types of operations that can be performed on caches
 * during their lifetime.
 */
enum class cache_op_src {
  ekEXISTING_CACHE_DROP,
  ekEXISTING_CACHE_PICKUP,
};

NS_END(tv, argos support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_SRC_HPP_ */
