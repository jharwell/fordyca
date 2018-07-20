/**
 * @file static_cache_params.hpp
 *
 * @copyright 2017h8 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_STATIC_CACHE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_STATIC_CACHE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"
#include "rcppsw/control/waveform_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);
namespace ct = rcppsw::control;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct static_cache_params
 * @ingroup params depth1
 */
struct static_cache_params : public rcppsw::params::base_params {
  /**
   * @brief Should the loop functions maintain a static cache?
   */
  bool enable{false};

  /**
   * @brief How large should the cache be, in terms of blocks?
   */
  uint size{0};

  /**
   * @brief When depleted, how quickly should the cache be re-created by the
   * arena. Very useful to debug a lot of the issues surrounding dynamic cache
   * creation in a more controlled setting.
   */
  double respawn_scale_factor{0.0};

  /**
   * @brief How large is the cache (geometrical area) ?
   */
  double dimension{0.0};

  /**
   * @brief How close do blocks have to be to each other to be considered "in"
   * the cache?
   */
  double min_dist{0.0};
  ct::waveform_params usage_penalty{};
};

NS_END(depth1, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_STATIC_CACHE_PARAMS_HPP_ */
