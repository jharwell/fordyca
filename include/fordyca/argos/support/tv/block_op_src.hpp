/**
 * \file block_op_src.hpp
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
 * \brief The different types of operations that can be perform on blocks.
 */
enum class block_op_src {
  ekFREE_PICKUP,
  ekNEST_DROP,
  ekCACHE_SITE_DROP,
  ekNEW_CACHE_DROP,
};

NS_END(tv, argos support, argos, fordyca);

