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
#include "fordyca/repr/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
nest::nest(const rmath::vector2d& dim,
           const rmath::vector2d& loc,
           double resolution,
           const rutils::color& light_color)
    : multicell_entity(dim, rcppsw::utils::color::kGRAY70),
      immovable_cell_entity(loc, resolution),
      m_lights(init_lights(light_color)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
nest::light_list nest::init_lights(const rutils::color& color) const {
  if (std::fabs(dims().x() - dims().y()) <=
      std::numeric_limits<double>::epsilon()) {
    return init_square(color);
  } else {
    return init_rect(color);
  }
} /* init_lights() */

nest::light_list nest::init_square(const rutils::color& color) const {
  argos::CVector3 loc(real_loc().x(), real_loc().y(), 5.0);
  return light_list{new argos::CLightEntity(
      "nest_light0",
      loc,
      argos::CColor(color.red(), color.green(), color.blue()),
      100.0)};
} /* init_square() */

nest::light_list nest::init_rect(const rutils::color& color) const {
  light_list ret;
  argos::CVector3 loc1, loc2, loc3;

  if (xdim() > ydim()) {
    loc1.Set(real_loc().x() - xdim() * 0.25, real_loc().y(), 5.0);
    loc2.Set(real_loc().x(), real_loc().y(), 5.0);
    loc3.Set(real_loc().x() + xdim() * 0.25, real_loc().y(), 5.0);
  } else {
    loc1.Set(real_loc().x(), real_loc().y() - ydim() * 0.25, 5.0);
    loc2.Set(real_loc().x(), real_loc().y(), 5.0);
    loc3.Set(real_loc().x(), real_loc().y() + ydim() * 0.25, 5.0);
  }

  return {new argos::CLightEntity(
              "nest_light0",
              loc1,
              argos::CColor(color.red(), color.green(), color.blue()),
              100.0),
          new argos::CLightEntity(
              "nest_light1",
              loc2,
              argos::CColor(color.red(), color.green(), color.blue()),
              100.0),
          new argos::CLightEntity(
              "nest_light2",
              loc3,
              argos::CColor(color.red(), color.green(), color.blue()),
              100.0)};
} /* init_rect() */

NS_END(repr, fordyca);
