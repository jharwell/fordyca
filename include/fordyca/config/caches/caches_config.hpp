/**
 * @file caches_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_CACHES_CACHES_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_CACHES_CACHES_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "fordyca/config/caches/static_cache_config.hpp"
#include "fordyca/config/caches/dynamic_cache_config.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, caches);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct caches_config
 * @ingroup fordyca config arena
 *
 * @brief Contains parameters for both static and dynamic caches in the arena.
 */
struct caches_config : public rconfig::base_config {
  /**
   * @brief How large are cache (geometrical area), when created (same for
   * static and dynamic) ?
   */
  rtypes::spatial_dist dimension{0.0};
  static_cache_config  static_{}; // NOLINT
  dynamic_cache_config dynamic{};
};

NS_END(arena, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_CACHE_CACHE_CONFIG_HPP_ */
