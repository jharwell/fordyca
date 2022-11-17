/**
 * \file caches_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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
  rspatial::euclidean_dist dimension{0.0};

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

