/**
 * \file cache_sel_matrix_config.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/range.hpp"
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/spatial_dist.hpp"

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
  rtypes::spatial_dist         cache_prox_dist{0.0};
  rtypes::spatial_dist         block_prox_dist{0.0};
  rtypes::spatial_dist         nest_prox_dist{0.0};
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

  rtypes::spatial_dist         new_cache_tol{0.0};
};

NS_END(cache_sel, config, controller, fordyca);

