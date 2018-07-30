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
/*
 * @todo Figure out how to remove this warning properly.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include "fordyca/support/depth0/stateless_foraging_qt_user_functions.hpp"
#pragma GCC diagnostic pop
#include <argos3/core/simulator/entity/controllable_entity.h>
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/support/block_carry_visualizer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_qt_user_functions::stateless_foraging_qt_user_functions() {
  RegisterUserFunction<stateless_foraging_qt_user_functions, argos::CFootBotEntity>(
      &stateless_foraging_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  auto& controller =
      dynamic_cast<controller::depth0::stateless_foraging_controller&>(
          c_entity.GetControllableEntity().GetController());

  if (controller.display_id()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.5), c_entity.GetId());
  }

  if (controller.is_carrying_block()) {
    block_carry_visualizer(this, kBLOCK_VIS_OFFSET, kTEXT_VIS_OFFSET)
        .draw(controller.block().get(), controller.GetId().size());
  }
}

using namespace argos;
REGISTER_QTOPENGL_USER_FUNCTIONS(stateless_foraging_qt_user_functions,
                                 "stateless_foraging_qt_user_functions"); // NOLINT

NS_END(depth0, support, fordyca);
