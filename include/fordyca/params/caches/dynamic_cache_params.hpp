/**
 * @file dynamic_cache_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_CACHES_DYNAMIC_CACHE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CACHES_DYNAMIC_CACHE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"
#include "rcppsw/control/waveform_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, caches);
namespace ct = rcppsw::control;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct dynamic_cache_params
 * @ingroup params cache
 */
struct dynamic_cache_params : public rcppsw::params::base_params {
  bool                enable{false};

  /**
   * @brief How close do blocks have to be to each other to be considered "in"
   * the cache?
   */
  double              min_dist{0.0};

 /**
   * @brief How many blocks does it take to create a dynamic cache?
   */
  uint                min_blocks{0};
};

NS_END(caches, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CACHES_DYNAMIC_CACHE_PARAMS_HPP_ */
