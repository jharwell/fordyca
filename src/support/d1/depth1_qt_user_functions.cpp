/**
 * \file depth1_qt_user_functions.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "rcppsw/common/common.hpp"

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_OVERLOADED_VIRTUAL()
#include "fordyca/support/d1/depth1_qt_user_functions.hpp"
RCPPSW_WARNING_DISABLE_POP()
#include "cosm/vis/task_visualizer.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d1);
namespace controller = controller::cognitive::d1;

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
  depth0_qt_user_functions::Draw(c_entity);

  auto& controller = dynamic_cast<controller::bitd_dpo_controller&>(
      c_entity.GetControllableEntity().GetController());

  if (controller.display_task()) {
    cvis::task_visualizer(this, 0.75)
        .draw(dynamic_cast<cta::logical_task*>(controller.current_task()));
  }
}

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_QTOPENGL_USER_FUNCTIONS(depth1_qt_user_functions,
                                 "depth1_qt_user_functions");
RCPPSW_WARNING_DISABLE_POP()

NS_END(support, fordyca, d1);
