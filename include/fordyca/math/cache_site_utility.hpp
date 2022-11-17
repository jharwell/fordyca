/**
 * \file cache_site_utility.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/sigmoid.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_site_utility
 * \ingroup math
 *
 * \brief Calculates the utility associated with a cache to a robot as part
 * of its decision process for what to do with a block once it has picked it up.
 *
 * Depends on:
 *
 * - Distance of perspective site to the nest (closer is better).
 * - Distance of perspective site to robot's current location (closer is
 * - better).
 *
 * The ideal location is halfway between the robot's current position and the
 * nest. However, if that location is unsuitable (i.e. it violates one or more
 * constraints), then we need to constrain our site selection to along the arc
 * of the circle formed by the nest center and distance to the ideal
 * point. We do this via exponential falloff on either side of the arc.
 */
class cache_site_utility : public rmath::sigmoid,
                           public rer::client<cache_site_utility> {
 public:
  cache_site_utility(const rmath::vector2d& position,
                     const rmath::vector2d& nest_loc);

  double calc(const rmath::vector2d& site_loc);
  double operator()(const rmath::vector2d& site_loc) { return calc(site_loc); }

 private:
  /* clang-format off */
  const rmath::vector2d mc_position;
  const rmath::vector2d mc_nest_loc;
  /* clang-format on */
};

NS_END(math, fordyca);
