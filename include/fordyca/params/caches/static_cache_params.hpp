/**
 * @file static_cache_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_CACHES_STATIC_CACHE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CACHES_STATIC_CACHE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, caches);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct static_cache_params
 * @ingroup fordyca params caches
 */
struct static_cache_params : public rparams::base_params {
  bool                enable{false};

  /**
   * @brief How large should the static cache be, in terms of blocks (if
   * enabled)?
   */
  uint                size{0};

  /**
   * @brief When depleted, how quickly should the cache be re-created by the
   * arena. Very useful to debug a lot of the issues surrounding dynamic cache
   * creation in a more controlled setting.
   */
  double              respawn_scale_factor{0.0};
};

NS_END(caches, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CACHES_STATIC_CACHE_PARAMS_HPP_ */
