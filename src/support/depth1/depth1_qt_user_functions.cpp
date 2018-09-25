/**
 * @file depth1_qt_user_functions.cpp
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
 * @todo Figure out how to work remove this warning properly.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include "fordyca/support/depth1/depth1_qt_user_functions.hpp"
#pragma GCC diagnostic pop
#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
namespace controller = controller::depth1;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_qt_user_functions::depth1_qt_user_functions(void) {
  RegisterUserFunction<depth1_qt_user_functions, argos::CFootBotEntity>(
      &depth1_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  stateful_qt_user_functions::Draw(c_entity);

  auto& controller = dynamic_cast<controller::greedy_partitioning_controller&>(
      c_entity.GetControllableEntity().GetController());

  if (controller.display_task() && nullptr != controller.current_task()) {
    DrawText(
        argos::CVector3(0.0, 0.0, 0.75),
        dynamic_cast<ta::executable_task*>(controller.current_task())->name(),
        argos::CColor::BLUE);
  }
}

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_QTOPENGL_USER_FUNCTIONS(depth1_qt_user_functions,
                                 "depth1_qt_user_functions");
#pragma Clang diagnostic pop

NS_END(support, fordyca, depth1);
