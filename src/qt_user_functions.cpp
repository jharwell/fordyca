/**
 * @file qt_user_functions.cpp
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
#include "fordyca/support/qt_user_functions.hpp"
#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
qt_user_functions::qt_user_functions() {
  RegisterUserFunction<qt_user_functions,
                       argos::CFootBotEntity>(&qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  controller::foraging_controller& controller = dynamic_cast<controller::foraging_controller&>(c_entity.GetControllableEntity().GetController());
  if (controller.carrying_block()) {
    DrawCylinder(
        argos::CVector3(0.0f, 0.0f, 0.3f),
        argos::CQuaternion(),
        0.1f,
        0.05f,
        argos::CColor::BLACK);
  }
}

using namespace argos;
REGISTER_QTOPENGL_USER_FUNCTIONS(qt_user_functions, "qt_user_functions")

NS_END(support, fordyca);
