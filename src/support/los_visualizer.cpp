/**
 * @file los_visualizer.cpp
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
#include "fordyca/support/los_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void los_visualizer::draw(const representation::line_of_sight* const los,
                          double grid_resolution) {
  /* has not been populated yet on first timestep */
  if (nullptr == los) {
    return;
  }
  std::vector<argos::CVector2> points;
  points.emplace_back(-grid_resolution * los->xsize() / 2,
                      -grid_resolution * los->ysize() / 2);
  points.emplace_back(-grid_resolution * los->xsize() / 2,
                      grid_resolution * los->ysize() / 2);
  points.emplace_back(grid_resolution * los->xsize() / 2,
                      grid_resolution * los->ysize() / 2);
  points.emplace_back(grid_resolution * los->xsize() / 2,
                      -grid_resolution * los->ysize() / 2);

  /* draw LOS slightly above the ground so that it renders better */
  m_qt->DrawPolygon(argos::CVector3(0, 0, 0.05),
                    argos::CQuaternion(),
                    points,
                    argos::CColor::RED,
                    false);
}

NS_END(support, fordyca);
