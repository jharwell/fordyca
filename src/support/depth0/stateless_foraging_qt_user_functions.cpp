/**
 * @file stateless_foraging_qt_user_functions.cpp
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
#include "fordyca/support/depth0/stateless_foraging_qt_user_functions.hpp"
#include <argos3/core/simulator/entity/controllable_entity.h>
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_qt_user_functions::stateless_foraging_qt_user_functions() {
  RegisterUserFunction<stateless_foraging_qt_user_functions,
                       argos::CFootBotEntity>(
      &stateless_foraging_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_qt_user_functions::Draw(
    argos::CFootBotEntity &c_entity) {
  auto &controller =
      dynamic_cast<controller::depth0::stateless_foraging_controller &>(
          c_entity.GetControllableEntity().GetController());

  if (controller.display_id()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.5), c_entity.GetId().c_str());
  }

  if (controller.is_carrying_block()) {
    DrawBox(argos::CVector3(0.0, 0.0, 0.3),
            argos::CQuaternion(),
            argos::CVector3(controller.block()->xsize(),
                            controller.block()->ysize(),
                            controller.block()->xsize()), /* assuming a cube */
            argos::CColor::BLACK);
    if (controller.block()->display_id()) {
      DrawText(argos::CVector3(0.0, 0.0, 0.5),
               std::string(controller.GetId().size()+3, ' ') +
               "[b" + std::to_string(controller.block()->id()) + "]",
               argos::CColor::GREEN);
    }
  }
}

using namespace argos;
REGISTER_QTOPENGL_USER_FUNCTIONS(stateless_foraging_qt_user_functions,
                                 "stateless_foraging_qt_user_functions");

NS_END(depth0, support, fordyca);
