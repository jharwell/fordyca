/**
 * \file bitd_odpo_controller.cpp
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
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"

#include "rcppsw/ta/bi_tdgraph_executive.hpp"

#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/oracular_info_receptor.hpp"
#include "fordyca/repr/base_block.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bitd_odpo_controller::bitd_odpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth1.bitd_odpo"),
      m_receptor(nullptr) {}

bitd_odpo_controller::~bitd_odpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bitd_odpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());

  dpo_perception()->update(m_receptor.get());
  executive()->run();
  saa()->steer_force2D_apply();
  ndc_pop();
} /* control_step() */

void bitd_odpo_controller::oracle_init(
    std::unique_ptr<oracular_info_receptor> receptor) {
  m_receptor = std::move(receptor);
  if (m_receptor->tasking_enabled()) {
    m_receptor->tasking_hooks_register(executive());
  }
} /* oracle_init() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(bitd_odpo_controller, "bitd_odpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth1, controller, fordyca);
