/**
 * @file cache_acq_point_selector.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_CACHE_ACQ_POINT_SELECTOR_HPP_
#define INCLUDE_FORDYCA_FSM_CACHE_ACQ_POINT_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class base_cache;
} /* namespace repr */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_acq_point_selector
 * @ingroup fordyca fsm
 *
 * @brief Once an existing cache has been selected, select a point within the
 * cache to acquire.
 *
 * This helps a LOT with maximimizing caches' potential for traffic/congestion
 * regulation, because it does not require that all robots be able to make it to
 * the center/near center of the cache in order to utilize it (this is more
 * realistic too).
 *
 * We also do not just want to pick a point randomly inside the ENTIRE cache,
 * as we might be trying to go to a point that is on the far side of the
 * cache, rather than the side that we are closest to. So we randomly pick a
 * point in the closest quadrant to the robot's current location.
 */
class cache_acq_point_selector : public rer::client<cache_acq_point_selector> {
 public:
  explicit cache_acq_point_selector(double arrival_tol)
      : ER_CLIENT_INIT("fordyca.fsm.cache_acq_point_selector"),
        m_arrival_tol(arrival_tol) {}

  rmath::vector2d operator()(const rmath::vector2d& robot_loc,
                             const repr::base_cache* cache,
                             rmath::rng* rng);

 private:
  /* clang-format off */
  const double m_arrival_tol;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_CACHE_ACQ_POINT_SELECTOR_HPP_ */
