/**
 * \file mech_malfunction.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "fordyca/events/mech_malfunction.hpp"

#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void mech_malfunction::visit(controller::foraging_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.supervisor());

  ER_INFO("Robot %s malfunctioned", controller.GetId().c_str());
  controller.ndc_pop();
} /* visit() */

void mech_malfunction::visit(cfsm::supervisor_fsm& fsm) {
  fsm.event_malfunction();
} /* visit() */

NS_END(detail, events, fordyca);
