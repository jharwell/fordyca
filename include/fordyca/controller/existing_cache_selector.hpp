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

#ifndef INCLUDE_FORDYCA_CONTROLLER_EXISTING_CACHE_TARGET_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_EXISTING_CACHE_TARGET_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>

#include "rcppsw/common/er_client.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class existing_cache_selector: public rcppsw::common::er_client {
 public:
  existing_cache_selector(
      const std::shared_ptr<rcppsw::common::er_server>& server,
      argos::CVector2 nest_loc);

  ~existing_cache_selector(void) { rmmod(); }

  /**
   * @brief Given a list of existing caches that a robot knows about (i.e. have
   * not faded into an unknown state), compute which is the "best", for use in
   * deciding which cache to go to and attempt to pickup from.
   *
   * @return A pointer to the "best" existing cache, along with its utility value.
   */
  representation::perceived_cache calc_best(
      const std::list<representation::perceived_cache> existing_caches,
      argos::CVector2 robot_loc);

 private:
  argos::CVector2 m_nest_loc;
};

NS_END(fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_EXISTING_CACHE_TARGET_SELECTOR_HPP_ */
