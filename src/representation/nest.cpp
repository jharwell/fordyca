/**
 * @file nest.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
nest::nest(const rmath::vector2d& dim,
           const rmath::vector2d& loc,
           double resolution)
    : multicell_entity(dim, rcppsw::utils::color::kGRAY70),
      immovable_cell_entity(loc, resolution),
      m_lights(init_lights()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
nest::light_list nest::init_lights(void) const {
  if (std::fabs(dims().x() - dims().y()) <=
      std::numeric_limits<double>::epsilon()) {
    return init_square();
  } else {
    return init_rect();
  }
} /* init_lights() */

nest::light_list nest::init_square(void) const {
  argos::CVector3 loc(real_loc().x(), real_loc().y(), 1.0);
  return light_list{
      new argos::CLightEntity("nest_light0", loc, argos::CColor::YELLOW, 10.0)};
} /* init_square() */

nest::light_list nest::init_rect(void) const {
  light_list ret;
  argos::CVector3 loc1, loc2, loc3;

  if (xdim() > ydim()) {
    loc1.Set(real_loc().x() - xdim() * 0.25, real_loc().y(), 1.0);
    loc2.Set(real_loc().x(), real_loc().y(), 1.0);
    loc3.Set(real_loc().x() + xdim() * 0.25, real_loc().y(), 1.0);
  } else {
    loc1.Set(real_loc().x(), real_loc().y() - ydim() * 0.25, 1.0);
    loc2.Set(real_loc().x(), real_loc().y(), 1.0);
    loc3.Set(real_loc().x(), real_loc().y() + ydim() * 0.25, 1.0);
  }
  return {
      new argos::CLightEntity("nest_light0", loc1, argos::CColor::YELLOW, 10.0),
      new argos::CLightEntity("nest_light1", loc2, argos::CColor::YELLOW, 10.0),
      new argos::CLightEntity("nest_light2", loc3, argos::CColor::YELLOW, 10.0)};
} /* init_rect() */

NS_END(representation, fordyca);
