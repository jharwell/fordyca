/**
 * @file existing_cache_selector.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_EXISTING_CACHE_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_EXISTING_CACHE_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <argos3/core/utility/math/vector2.h>

#include "rcppsw/er/client.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
class cache_sel_matrix;
NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class existing_cache_selector
 * @ingroup controller depth1
 *
 * @brief Selects from among known caches (which are presumed to still exist at
 * this point, although that may not be true as a robot's knowledge of the arena
 * is imperfect).
 */
class existing_cache_selector: public rcppsw::er::client<existing_cache_selector> {
 public:
  explicit existing_cache_selector(const cache_sel_matrix* matrix);

  ~existing_cache_selector(void) override = default;
  existing_cache_selector& operator=(const existing_cache_selector& other) = delete;
  existing_cache_selector(const existing_cache_selector& other) = delete;

  /**
   * @brief Given a list of existing caches that a robot knows about (i.e. have
   * not faded into an unknown state), compute which is the "best", for use in
   * deciding which cache to go to and attempt to pickup from.
   *
   * @return The "best" existing cache.
   */
  representation::perceived_cache calc_best(
      const std::list<representation::perceived_cache>& existing_caches,
      argos::CVector2 robot_loc);

 private:
  const cache_sel_matrix* const mc_matrix;
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_EXISTING_CACHE_SELECTOR_HPP_ */
