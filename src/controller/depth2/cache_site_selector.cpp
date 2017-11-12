/**
 * @file cache_site_selector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth2/cache_site_selector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_selector::cache_site_selector(
    const std::shared_ptr<rcppsw::common::er_server>& server,
    argos::CVector2 nest_loc) :
    er_client(server),
    m_nest_loc(nest_loc) {
  er_client::insmod("cache_site_selector",
                    rcppsw::common::er_lvl::DIAG,
                    rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos::CVector2 cache_site_selector::calc_best(
    const std::list<representation::perceived_cache>,
    argos::CVector2 robot_loc) {

  argos::CVector2 site((robot_loc.GetX() - m_nest_loc.GetX()) / 2.0,
                       m_nest_loc.GetY());

  ER_NOM("Best utility: cache_site at (%f, %f)", site.GetX(), site.GetY());
  return site;
} /* calc_best() */

NS_END(controller, fordyca);
