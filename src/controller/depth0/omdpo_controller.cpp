/**
 * \file omdpo_controller.cpp
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
#include "fordyca/controller/depth0/omdpo_controller.hpp"

#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem2D.hpp"

#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
omdpo_controller::omdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.omdpo"), m_receptor(nullptr) {}

omdpo_controller::~omdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void omdpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->md()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  mdpo_perception()->update(m_receptor.get());
  fsm()->run();
  saa()->steer_force2D_apply();
  ndc_pop();
} /* control_step() */

void omdpo_controller::oracle_init(
    std::unique_ptr<oracular_info_receptor> receptor) {
  m_receptor = std::move(receptor);
} /* oracle_init() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(omdpo_controller, "omdpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth0, controller, fordyca);
