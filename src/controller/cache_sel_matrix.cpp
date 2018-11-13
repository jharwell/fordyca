/**
 * @file cache_sel_matrix.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/params/cache_sel_matrix_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
constexpr char cache_sel_matrix::kNestLoc[];
constexpr char cache_sel_matrix::kCacheProxDist[];
constexpr char cache_sel_matrix::kBlockProxDist[];
constexpr char cache_sel_matrix::kNestProxDist[];
constexpr char cache_sel_matrix::kSiteXRange[];
constexpr char cache_sel_matrix::kSiteYRange[];

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_sel_matrix::cache_sel_matrix(
    const struct params::cache_sel_matrix_params* const params,
    const rmath::vector2d& nest_loc) {
  this->insert(std::make_pair(kNestLoc, nest_loc));
  this->insert(std::make_pair(kCacheProxDist, params->cache_prox_dist));
  this->insert(std::make_pair(kBlockProxDist, params->block_prox_dist));
  this->insert(std::make_pair(kNestProxDist, params->nest_prox_dist));
  this->insert(std::make_pair(kSiteXRange, params->site_xrange));
  this->insert(std::make_pair(kSiteYRange, params->site_yrange));
}

NS_END(controller, fordyca);
