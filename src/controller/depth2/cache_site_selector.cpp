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
#include "fordyca/controller/cache_selection_matrix.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_site_selector::cache_site_selector(
    std::shared_ptr<rcppsw::er::server> server,
    const controller::cache_selection_matrix* const matrix)
    : client(server), mc_matrix(matrix) {
  client::insmod("cache_site_selector",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos::CVector2 cache_site_selector::calc_best(
    const std::list<representation::perceived_cache>&,
    argos::CVector2 robot_loc) {
  argos::CVector2 nest_loc =
      boost::get<argos::CVector2>(mc_matrix->find("nest_center")->second);
  argos::CVector2 site((robot_loc.GetX() - nest_loc.GetX()) / 2.0,
                       nest_loc.GetY());

  ER_NOM("Best utility: cache_site at (%f, %f)", site.GetX(), site.GetY());
  return site;
} /* calc_best() */

NS_END(depth2, controller, fordyca);
