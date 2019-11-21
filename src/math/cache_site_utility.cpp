/**
 * \file cache_site_utility.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/math/cache_site_utility.hpp"

#include <cmath>
#include <limits>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_utility::cache_site_utility(const rmath::vector2d& position,
                                       const rmath::vector2d& nest_loc)
    : sigmoid(4.0, 1.0, 1.0),
      ER_CLIENT_INIT("fordyca.math.cache_site_utility"),
      mc_position(position),
      mc_nest_loc(nest_loc) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double cache_site_utility::calc(const rmath::vector2d& site_loc) {
  rmath::vector2d ideal_loc = (mc_position + mc_nest_loc) / 2;
  double deviation_from_ideal = (site_loc - ideal_loc).length();
  double theta = reactivity() * (deviation_from_ideal - offset());
  double deviation_scaling = gamma() * 1.0 / (1.0 + std::exp(theta));
  ER_TRACE("ideal_loc=%s, point=%s, deviation=%f, deviation_scaling=%f",
           ideal_loc.to_str().c_str(),
           site_loc.to_str().c_str(),
           deviation_from_ideal,
           deviation_scaling);

  /*
   * If we don't impose a minimum distance the cache site has to be from the
   * robot, then if a robot's initial guess of the cache site is right where it
   * currently is (i.e. where it just picked up a block from), then it can get a
   * high utility for that spot and choose it to drop the block in. If it then
   * allocates itself the Cache Starter task again, you can get stuck in an
   * infinite loop.
   */
  double dist_to_robot = std::max(1.0, (site_loc - mc_position).length());
  double dist_to_nest = (site_loc - ideal_loc).length();
  ER_TRACE("Utility: %f",
           (1.0 / (dist_to_robot * dist_to_nest)) * deviation_scaling);
  if (deviation_from_ideal <= std::numeric_limits<double>::epsilon()) {
    return eval(std::numeric_limits<double>::min());
  }
  return eval((1.0 / (dist_to_robot * dist_to_nest)) * deviation_scaling);
} /* calc() */

NS_END(expressions, fordyca);
