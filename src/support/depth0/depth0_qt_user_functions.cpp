/**
 * @file depth0_qt_user_functions.cpp
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
#include "fordyca/support/depth0/depth0_qt_user_functions.hpp"
#pragma GCC diagnostic pop

#include <argos3/core/simulator/entity/controllable_entity.h>
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/support/block_carry_visualizer.hpp"
#include "fordyca/support/los_visualizer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth0_qt_user_functions::depth0_qt_user_functions(void) {
  RegisterUserFunction<depth0_qt_user_functions, argos::CFootBotEntity>(
      &depth0_qt_user_functions::Draw);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth0_qt_user_functions::Draw(argos::CFootBotEntity& c_entity) {
  auto* mdpo = dynamic_cast<const controller::depth0::mdpo_controller*>(
      &c_entity.GetControllableEntity().GetController());
  auto* crw = dynamic_cast<const controller::depth0::crw_controller*>(
      &c_entity.GetControllableEntity().GetController());

  if (crw->display_id()) {
    DrawText(argos::CVector3(0.0, 0.0, 0.5), c_entity.GetId());
  }

  if (crw->is_carrying_block()) {
    block_carry_visualizer(this, kBLOCK_VIS_OFFSET, kTEXT_VIS_OFFSET)
        .draw(crw->block().get(), crw->GetId().size());
  }
  if (nullptr != mdpo && mdpo->display_los()) {
    los_visualizer(this).draw(mdpo->los(),
                              mdpo->mdpo_perception()->map()->grid_resolution());
  }
}

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_QTOPENGL_USER_FUNCTIONS(depth0_qt_user_functions,
                                 "depth0_qt_user_functions"); // NOLINT
#pragma clang diagnostic pop

NS_END(depth0, support, fordyca);
