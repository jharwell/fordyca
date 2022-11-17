/**
 * \file arrival_tol.hpp
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

/**
 * \brief The tolerance within which a robot's location has to be in order to
 * be considered having arrived at the specified block's location.
 */
static constexpr double kBLOCK_ARRIVAL_TOL = 0.02;

/**
 * \brief The tolerance within which a robot's location has to be in order to
 * be considered having arrived at the specified cache's location.
 */
static constexpr double kCACHE_ARRIVAL_TOL = 0.02;

/**
 * \brief The tolerance within which a robot's location has to be in order to
 * be considered having arrived at the specified new cache's location. More
 * relaxed than the tolerance for cache arrivals, as we are creating a cache,
 * and things can be much more approximate.
 */
static constexpr double kNEW_CACHE_ARRIVAL_TOL = 0.8;

/**
 * \brief The tolerance within which a robot's location has to be in order to
 * be considered to have arrived at the specified cache site.
 */
static constexpr double kCACHE_SITE_ARRIVAL_TOL = 0.8;

NS_END(fordyca, fsm);
