/**
 * \file cache_op_src.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
 * \brief The different types of operations that can be performed on caches
 * during their lifetime.
 */
enum class cache_op_src {
  ekEXISTING_CACHE_DROP,
  ekEXISTING_CACHE_PICKUP,
};

NS_END(tv, argos support, argos, fordyca);

