/**
 * \file birtd_odpo_controller.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"

#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/oracular_info_receptor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
birtd_odpo_controller::birtd_odpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.birtd_dpo"),
      m_receptor(nullptr) {}

birtd_odpo_controller::~birtd_odpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void birtd_odpo_controller::control_step(void) {
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
  saa()->apf().tracking_reset();

  /*
   * Execute the current task/allocate a new task/abort a task/etc and apply
   * steering forces if normal operation, otherwise handle abnormal operation
   * state.
   */
  supervisor()->run();

  /* Update block detection status for use in the loop functions */
  block_detect_status_update();

  ndc_uuid_pop();
} /* control_step() */

void birtd_odpo_controller::oracle_init(
    std::unique_ptr<fsperception::oracular_info_receptor> receptor) {
  m_receptor = std::move(receptor);
  if (m_receptor->tasking_enabled()) {
    m_receptor->tasking_hooks_register(executive());
  }
} /* oracle_init() */

NS_END(cognitive, d2, controller, fordyca);

using namespace fccd2; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(birtd_odpo_controller, "birtd_odpo_controller"); // NOLINT

RCPPSW_WARNING_DISABLE_POP()
