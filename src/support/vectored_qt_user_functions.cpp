/**
 * @file vectored_qt_user_functions.cpp
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
#include <argos3/core/simulator/entity/controllable_entity.h>
#include "fordyca/support/vectored_qt_user_functions.hpp"
#include "fordyca/controller/vectored_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
vectored_qt_user_functions::vectored_qt_user_functions() {
  RegisterUserFunction<vectored_qt_user_functions,
                       argos::CFootBotEntity>(&vectored_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void vectored_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  base_qt_user_functions::Draw(c_entity);

  controller::vectored_controller& controller =
      dynamic_cast<controller::vectored_controller&>(
          c_entity.GetControllableEntity().GetController());
  if (controller.display_los()) {
    const representation::line_of_sight* los = controller.los();
    DrawCircle(argos::CVector3(0, 0, 0),
               argos::CQuaternion(),
               (los->sizex()/2)*0.2,
               argos::CColor::RED,
               false);
  }
}

using namespace argos;
REGISTER_QTOPENGL_USER_FUNCTIONS(vectored_qt_user_functions, "vectored_qt_user_functions")

NS_END(support, fordyca);
