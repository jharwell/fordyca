/**
 * @file energy_max_controller.cpp
 *
 * @copyright 2018 Anthony Chen/John Harwell, All rights reserved.
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
#include "fordyca/controller/saa_subsystem.hpp"


 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, controller, depth0);

 /*******************************************************************************
  * Constructors/Destructor
  ******************************************************************************/
 energy_max_controller::energy_max_controller(void)
     : stateless_foraging_controller(), m_fsm() {}

 energy_max_controller::~energy_max_controller(void) = default;

 /*******************************************************************************
  * Member Functions
  ******************************************************************************/
 void energy_max_controller::Init(argos::TConfigurationNode& node) {
   base_foraging_controller::Init(node);

   ER_NOM("Initializing energy_maximization_foraging controller");

   params::depth0::stateless_foraging_repository param_repo;
   param_repo.parse_all(node);
   param_repo.show_all(client::server_handle()->log_stream());
   ER_ASSERT(param_repo.validate_all(),
             "FATAL: Not all parameters were validated");

   m_fsm = rcppsw::make_unique<fsm::depth0::stateless_foraging_fsm>(
       static_cast<const struct params::fsm_params*>(
           param_repo.get_params("fsm")),
       base_foraging_controller::server(),
       base_foraging_controller::base_sensors_ref(),
       base_foraging_controller::actuators());
   ER_NOM("stateless_foraging controller initialization finished");
 } /* Init() */

 void energy_max_controller::Reset(void) {
   base_foraging_controller::Reset();
   if (nullptr != m_fsm) {
     m_fsm->init();
   }
 } /* Reset() */

 void energy_max_controller::ControlStep(void) {
   saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
   saa_subsystem()->actuation()->block_throttle_update();
   m_fsm->run();
 } /* ControlStep() */

 /*******************************************************************************
  * Add Metrics
  ******************************************************************************/


 /* Notifiy ARGoS of the existence of the controller. */
 using namespace argos;
 REGISTER_CONTROLLER(energy_max_controller,
                     "energy_max_controller"); // NOLINT

 NS_END(depth0, controller, fordyca);
