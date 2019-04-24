/**
 * @file grp_omdpo_controller.cpp
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
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/repr/base_block.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
grp_omdpo_controller::grp_omdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth2.grp_dpo"),
      m_receptor(nullptr) {}

grp_omdpo_controller::~grp_omdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grp_omdpo_controller::ControlStep(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());
  mdpo_perception()->update(m_receptor.get());

  executive()->run();
  ndc_pop();
} /* ControlStep() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(grp_omdpo_controller, "grp_omdpo_controller"); // NOLINT
#pragma clang diagnostic pop

NS_END(depth2, controller, fordyca);
