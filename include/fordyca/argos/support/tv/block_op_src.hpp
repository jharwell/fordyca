/**
 * \file block_op_src.hpp
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

#pragma once

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
 * \brief The different types of operations that can be perform on blocks.
 */
enum class block_op_src {
  ekFREE_PICKUP,
  ekNEST_DROP,
  ekCACHE_SITE_DROP,
  ekNEW_CACHE_DROP,
};

NS_END(tv, argos support, argos, fordyca);

