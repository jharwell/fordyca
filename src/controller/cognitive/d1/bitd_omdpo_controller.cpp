/**
 * \file bitd_omdpo_controller.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"

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
NS_START(fordyca, controller, cognitive, d1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bitd_omdpo_controller::bitd_omdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.bitd_omdpo"), m_receptor(nullptr) {}

bitd_omdpo_controller::~bitd_omdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bitd_omdpo_controller::control_step(void) {
  mdc_ts_update();
  ndc_uuid_push();
  ER_ASSERT(!(nullptr != block() && !block()->is_carried_by_robot()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  perception()->update(m_receptor.get());

  /*
   * Reset steering forces tracking so per-timestep visualizations are
   * correct. This can't be done when applying the steering forces because then
   * they are always 0 during loop function visualization.
   */
  saa()->steer_force2D().tracking_reset();

  /*
   * Execute the current task/allocate a new task/abort a task/etc and apply
   * steering forces if normal operation, otherwise handle abnormal operation
   * state.
   */
  supervisor()->run();

  ndc_uuid_pop();
} /* control_step() */

void bitd_omdpo_controller::oracle_init(
    std::unique_ptr<fsperception::oracular_info_receptor> receptor) {
  m_receptor = std::move(receptor);
} /* oracle_init() */

NS_END(cognitive, d1, controller, fordyca);

using namespace fccd1; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(bitd_omdpo_controller, "bitd_omdpo_controller");

RCPPSW_WARNING_DISABLE_POP()
