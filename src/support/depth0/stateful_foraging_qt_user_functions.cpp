/**
 * @file foraging_qt_user_functions.cpp
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
/*
 * @todo Figure out how to remove this warning properly.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include "fordyca/support/depth0/stateful_foraging_qt_user_functions.hpp"
#pragma GCC diagnostic pop

#include <argos3/core/simulator/entity/controllable_entity.h>
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_foraging_qt_user_functions::stateful_foraging_qt_user_functions() {
  RegisterUserFunction<stateful_foraging_qt_user_functions, argos::CFootBotEntity>(
      &stateful_foraging_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_foraging_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  stateless_foraging_qt_user_functions::Draw(c_entity);

  auto& controller =
      dynamic_cast<controller::depth0::stateful_foraging_controller&>(
          c_entity.GetControllableEntity().GetController());

  if (controller.display_los()) {
    const representation::line_of_sight* los = controller.los();
    const double resolution = controller.perception()->map()->grid_resolution();
    std::vector<argos::CVector2> points;
    points.emplace_back(-resolution * los->xsize() / 2,
                        -resolution * los->ysize() / 2);
    points.emplace_back(-resolution * los->xsize() / 2,
                        resolution * los->ysize() / 2);
    points.emplace_back(resolution * los->xsize() / 2,
                        resolution * los->ysize() / 2);
    points.emplace_back(resolution * los->xsize() / 2,
                        -resolution * los->ysize() / 2);

    /* draw LOS slightly above the ground so that it renders better */
    DrawPolygon(argos::CVector3(0, 0, 0.05),
                argos::CQuaternion(),
                points,
                argos::CColor::RED,
                false);
  }
}

using namespace argos;
REGISTER_QTOPENGL_USER_FUNCTIONS(stateful_foraging_qt_user_functions,
                                 "stateful_foraging_qt_user_functions"); // NOLINT

NS_END(depth0, support, fordyca);
