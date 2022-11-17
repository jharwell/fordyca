/**
 * \file static_cache_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/config/base_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct static_cache_config
 * \ingroup argos support caches config
 *
 * \brief Configuration for static caches in the arena for the loop functions.
 */
struct static_cache_config final : public rconfig::base_config {
  bool                enable{false};

  /**
   * \brief How large should the static cache be, in terms of blocks (if
   * enabled)?
   */
  uint                size{0};

  /**
   * \brief When depleted, how quickly should the cache be re-created by the
   * arena. Very useful to debug a lot of the issues surrounding dynamic cache
   * creation in a more controlled setting.
   */
  double              respawn_scale_factor{0.0};
};

NS_END(config, caches, support, argos, fordyca);

