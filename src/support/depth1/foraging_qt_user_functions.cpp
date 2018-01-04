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
#include "fordyca/support/depth1/foraging_qt_user_functions.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_qt_user_functions::foraging_qt_user_functions(void) {
  RegisterUserFunction<foraging_qt_user_functions,
                       argos::CFootBotEntity>(
      &foraging_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_qt_user_functions::Draw(argos::CFootBotEntity &c_entity) {
  stateful_foraging_qt_user_functions::Draw(c_entity);

  auto &controller =
      dynamic_cast<controller::depth1::foraging_controller &>(
          c_entity.GetControllableEntity().GetController());

  if (controller.display_task()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.75), controller.task_name(), argos::CColor::BLUE);
  }
}

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
typedef foraging_qt_user_functions depth1_foraging_qt_user_functions;
REGISTER_QTOPENGL_USER_FUNCTIONS(foraging_qt_user_functions,
                                 "depth1_foraging_qt_user_functions")

NS_END(support, fordyca, depth1);
