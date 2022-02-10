/**
 * \file arrival_tol.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

