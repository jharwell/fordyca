/**
 * @file crw_controller.cpp
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
#include "fordyca/controller/depth0/crw_controller.hpp"
#include <fstream>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/expstrat/factory.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw_controller::crw_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.crw"), m_fsm() {}

crw_controller::~crw_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void crw_controller::Init(ticpp::Element& node) {
  base_controller::Init(node);
  ndc_push();
  ER_INFO("Initializing...");

  fsm::expstrat::base_expstrat::params p = {.saa = saa_subsystem(),
                                            .store = nullptr};
  m_fsm = rcppsw::make_unique<fsm::depth0::crw_fsm>(
      saa_subsystem(),
      fsm::expstrat::factory().create(fsm::expstrat::factory::kCRWBlock, &p));
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void crw_controller::Reset(void) {
  base_controller::Reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* Reset() */

void crw_controller::ControlStep(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());
  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_OVERRIDE_DEF(bool, crw_controller, goal_acquired, *m_fsm, const);

FSM_OVERRIDE_DEF(bool, crw_controller, is_exploring_for_goal, *m_fsm, const);

FSM_OVERRIDE_DEF(acquisition_goal_type,
                 crw_controller,
                 acquisition_goal,
                 *m_fsm,
                 const);

FSM_OVERRIDE_DEF(transport_goal_type,
                 crw_controller,
                 block_transport_goal,
                 *m_fsm,
                 const);

FSM_OVERRIDE_DEF(rmath::vector2u, crw_controller, acquisition_loc, *m_fsm, const);
FSM_OVERRIDE_DEF(rmath::vector2u,
                 crw_controller,
                 current_vector_loc,
                 *m_fsm,
                 const);
FSM_OVERRIDE_DEF(rmath::vector2u,
                 crw_controller,
                 current_explore_loc,
                 *m_fsm,
                 const);

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(crw_controller, "crw_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
