/**
 * @file cache_sel_matrix.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_MATRIX_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_MATRIX_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <boost/variant.hpp>
#include <map>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/range.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
struct cache_sel_matrix_params;
}
NS_START(controller);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class cache_sel_matrix
 * @ingroup controller
 *
 * @brief A dictionary of information needed by robots using various utility
 * functions to calculate the best:
 *
 * - existing cache
 * - new cache
 * - cache site
 *
 * This class may be separated into those components in the future if it makes
 * sense. For now, it is cleaner to have all three uses be in the same class.
 */
class cache_sel_matrix
    : public std::map<std::string, boost::variant<double, argos::CVector2,
                                                  rmath::range<uint>>> {
 public:
  static constexpr char kNestLoc[] = "nest_loc";
  static constexpr char kCacheProxDist[] = "cache_prox_dist";
  static constexpr char kBlockProxDist[] = "block_prox_dist";
  static constexpr char kNestProxDist[] = "nest_prox_dist";
  static constexpr char kSiteXRange[] = "site_xrange";
  static constexpr char kSiteYRange[] = "site_yrange";

  cache_sel_matrix(const struct params::cache_sel_matrix_params* params,
                         const argos::CVector2& nest_loc);
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_CACHE_SEL_MATRIX_HPP_ */
