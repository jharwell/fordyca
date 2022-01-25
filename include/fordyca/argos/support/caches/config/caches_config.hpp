/**
 * \file caches_config.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_CACHES_CONFIG_CACHES_CONFIG_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_CACHES_CONFIG_CACHES_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "fordyca/argos/support/caches/config/static_cache_config.hpp"
#include "fordyca/argos/support/caches/config/dynamic_cache_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct caches_config
 * \ingroup argos support caches config
 *
 * \brief Contains parameters for both static and dynamic caches in the arena.
 */
struct caches_config final : public rconfig::base_config {
  /**
   * \brief How large are cache (geometrical area), when created (same for
   * static and dynamic) ?
   */
  rtypes::spatial_dist dimension{0.0};

  /**
   * \brief If \c TRUE, then cache creation will be strict, meaning that any
   * dynamically created caches that fail validation after creation will be
   * discarded, and any statically created caches that fail validation will
   * trigger an assert.
   *
   * If \c FALSE, then all caches will be kept after creation.
   */
  bool   strict_constraints{true};

  static_cache_config  static_{}; // NOLINT
  dynamic_cache_config dynamic{};
};

NS_END(config, arena, support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_CACHES_CONFIG_CACHES_CONFIG_HPP_ */
