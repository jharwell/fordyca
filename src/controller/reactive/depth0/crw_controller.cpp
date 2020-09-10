/**
 * \file crw_controller.cpp
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
#include "fordyca/controller/reactive/depth0/crw_controller.hpp"

#include <fstream>

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, reactive, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw_controller::crw_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.crw"), m_fsm() {}

crw_controller::~crw_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void crw_controller::init(ticpp::Element& node) {
  foraging_controller::init(node);
  ndc_push();
  ER_INFO("Initializing...");

  fsm::expstrat::foraging_expstrat::params p(saa(),
                                             nullptr,
                                             nullptr,
                                             nullptr,
                                             rutils::color());
  m_fsm = std::make_unique<fsm::depth0::crw_fsm>(
      saa(),
      fsm::expstrat::block_factory().create(
          fsm::expstrat::block_factory::kCRW, &p, rng()),
      rng());
  /* Set CRW FSM supervision */
  supervisor()->supervisee_update(m_fsm.get());
  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void crw_controller::reset(void) {
  foraging_controller::reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* reset() */

void crw_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->md()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();
  ndc_pop();
} /* control_step() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, goal_acquired, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, entity_acquired_id, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, is_exploring_for_goal, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, acquisition_goal, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, block_transport_goal, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, acquisition_loc3D, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, vector_loc3D, *m_fsm, const);
RCPPSW_WRAP_OVERRIDE_DEF(crw_controller, explore_loc3D, *m_fsm, const);

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(crw_controller, "crw_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(reactive, depth0, controller, fordyca);
