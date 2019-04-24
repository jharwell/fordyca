/**
 * @file caches_params.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_CACHES_CACHES_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CACHES_CACHES_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/base_params.hpp"
#include "fordyca/params/caches/static_cache_params.hpp"
#include "fordyca/params/caches/dynamic_cache_params.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, caches);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct caches_params
 * @ingroup fordyca params arena
 *
 * @brief Contains parameters for both static and dynamic caches in the arena.
 */
struct caches_params : public rparams::base_params {
  /**
   * @brief How large are cache (geometrical area), when created (same for
   * static and dynamic) ?
   */
  double               dimension{0.0};
  static_cache_params  static_{}; // NOLINT
  dynamic_cache_params dynamic{};
};

NS_END(arena, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CACHE_CACHE_PARAMS_HPP_ */
