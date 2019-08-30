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

#include "fordyca/config/cache_sel/cache_sel_matrix_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_sel_matrix::cache_sel_matrix(
    const config::cache_sel::cache_sel_matrix_config* const config,
    const rmath::vector2d& nest_loc)
    : ER_CLIENT_INIT("fordyca.controller.cache_sel_matrix") {
  this->insert(std::make_pair(kNestLoc, nest_loc));
  this->insert(std::make_pair(kCacheProxDist, config->cache_prox_dist));
  this->insert(std::make_pair(kBlockProxDist, config->block_prox_dist));
  this->insert(std::make_pair(kNestProxDist, config->nest_prox_dist));
  this->insert(std::make_pair(kClusterProxDist, config->nest_prox_dist));
  this->insert(std::make_pair(kSiteXRange, config->site_xrange));
  this->insert(std::make_pair(kSiteYRange, config->site_yrange));
  this->insert(std::make_pair(kPickupExceptions, std::vector<int>()));
  this->insert(std::make_pair(kDropExceptions, std::vector<int>()));
  this->insert(std::make_pair(kPickupPolicy, config->pickup_policy));
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_sel_matrix::sel_exception_add(const cache_sel_exception& ex) {
  switch (ex.type) {
    case cache_sel_exception::kPickup: {
      auto& vec =
          boost::get<std::vector<int>>(this->find(kPickupExceptions)->second);
      vec.push_back(ex.id);
    } break;
    case cache_sel_exception::kDrop: {
      auto& vec =
          boost::get<std::vector<int>>(this->find(kDropExceptions)->second);
      vec.push_back(ex.id);
    } break;
    default:
      ER_FATAL_SENTINEL("Bad exception type %d", ex.type);
  } /* switch() */
} /* sel_exception_add() */

void cache_sel_matrix::sel_exceptions_clear(void) {
  boost::get<std::vector<int>>(this->at(kPickupExceptions)).clear();
  boost::get<std::vector<int>>(this->at(kDropExceptions)).clear();
} /* sel_exceptions_clear() */

NS_END(controller, fordyca);
