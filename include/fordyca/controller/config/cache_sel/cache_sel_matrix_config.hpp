/**
 * \file cache_sel_matrix_config.hpp
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

#include "rcppsw/math/range.hpp"
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "fordyca/controller/config/cache_sel/cache_pickup_policy_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct cache_sel_matrix_config
 * \ingroup controller config cache_sel
 *
 * \brief XML parameters for the \ref cache_sel_matrix
 */
struct cache_sel_matrix_config final : public rconfig::base_config {
  rspatial::euclidean_dist         cache_prox_dist{0.0};
  rspatial::euclidean_dist         block_prox_dist{0.0};
  rspatial::euclidean_dist         nest_prox_dist{0.0};
  rmath::rangez                site_xrange{};
  rmath::rangez                site_yrange{};
  cache_pickup_policy_config   pickup_policy{};

  /**
   *  \brief If \c TRUE the site returned by the cache site selector violates
   * any constraints, it is discarded, which will lead to the robot trying again
   * the following timestep; this is more expensive computationally. If \c
   * FALSE, then if site returned by the cache site selector violates any
   * constraints, it is still returned as the "best-effort" site found; this is
   * cheaper computationally.
   */
  bool                         strict_constraints{true};

  rspatial::euclidean_dist         new_cache_tol{0.0};
};

NS_END(cache_sel, config, controller, fordyca);

