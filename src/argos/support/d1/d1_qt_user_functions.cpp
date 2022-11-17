/**
 * \file d1_qt_user_functions.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_OVERLOADED_VIRTUAL()
#include "fordyca/argos/support/d1/d1_qt_user_functions.hpp"
RCPPSW_WARNING_DISABLE_POP()
#include "cosm/argos/vis/task_visualizer.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d1);
namespace controller = controller::cognitive::d1;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
d1_qt_user_functions::d1_qt_user_functions(void) {
  RegisterUserFunction<d1_qt_user_functions, chal::robot>(
      &d1_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void d1_qt_user_functions::Draw(chal::robot& c_entity) {
  d0_qt_user_functions::Draw(c_entity);

  auto& controller = dynamic_cast<controller::bitd_dpo_controller&>(
      c_entity.GetControllableEntity().GetController());

  if (controller.display_task()) {
    cavis::task_visualizer(this, 0.75)
        .draw(dynamic_cast<cta::logical_task*>(controller.current_task()));
  }
}

NS_END(support, fordyca, argos, d1);

using namespace fasd1; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_QTOPENGL_USER_FUNCTIONS(d1_qt_user_functions, "d1_qt_user_functions");
RCPPSW_WARNING_DISABLE_POP()
