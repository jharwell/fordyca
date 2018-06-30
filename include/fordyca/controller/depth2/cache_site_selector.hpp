/**
 * @file cache_site_selector.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <argos3/core/utility/math/vector2.h>

#include "rcppsw/er/client.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_site_selector
 * @ingroup depth2
 *
 * @brief Selects the best cache site between the location of the block pickup
 * and the nest (ideally the halfway point), subject to constraints such as it
 * can't be too near other known caches.
 */
class cache_site_selector: public rcppsw::er::client {
 public:
  cache_site_selector(
      const std::shared_ptr<rcppsw::er::server>& server,
      argos::CVector2 nest_loc);

  ~cache_site_selector(void) override { client::rmmod(); }

  /**
   * @brief Given a list of existing caches that a robot knows about (i.e. have
   * not faded into an unknown state), compute the best site for a new cache.
   *
   * @return A pointer to the "best" cache site, along with its utility value.
   */
  argos::CVector2 calc_best(
      const std::list<representation::perceived_cache>& known_caches,
      argos::CVector2 robot_loc);

 private:
  argos::CVector2 m_nest_loc;
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_CACHE_SITE_SELECTOR_HPP_ */
