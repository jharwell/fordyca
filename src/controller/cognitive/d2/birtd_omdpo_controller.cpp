/**
 * \file birtd_omdpo_controller.cpp
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
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ds/cell2D.hpp"

#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/oracular_info_receptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
birtd_omdpo_controller::birtd_omdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.d2.birtd_dpo"), m_receptor(nullptr) {}

birtd_omdpo_controller::~birtd_omdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void birtd_omdpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && !block()->is_carried_by_robot()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());
  perception()->update(m_receptor.get());

  /*
   * Execute the current task/allocate a new task/abort a task/etc and apply
   * steering forces if normal operation, otherwise handle abnormal operation
   * state.
   */
  supervisor()->run();

  ndc_pop();
} /* control_step() */

void birtd_omdpo_controller::oracle_init(
    std::unique_ptr<fsperception::oracular_info_receptor> receptor) {
  m_receptor = std::move(receptor);
} /* oracle_init() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(birtd_omdpo_controller, "birtd_omdpo_controller"); // NOLINT

RCPPSW_WARNING_DISABLE_POP()

NS_END(cognitive, d2, controller, fordyca);
