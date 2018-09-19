/**
 * @file stateless_foraging_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_ENERGY_MAX_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_ENERGY_MAX_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "rcppsw/robotics/hal/sensors/battery_sensor.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace fsm { namespace depth0 { class stateless_foraging_fsm; } }

NS_START(controller, depth0);
namespace er = rcppsw::er;
namespace sensor rcppsw::robotics::hal::sensors;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class energy_max_controller
 * @ingroup controller depth0
 *
 * @brief foraging controller that uses different behaviors to maximize energy
 * efficiency.
 */

class energy_max_controller : public stateless_foraging_controller {


  public:
    energy_max_controller(void);
    ~energy_max_controller(void);  //override?

    /* CCI_Controller overrides */
    void Init(argos::TConfigurationNode& node) override;
    void ControlStep(void) override;
    void Reset(void) override;

    /* Energy functions */
    void sense_energy(void) { energy_level = bs.readings().time_left}

  private:
    // put in a energy params struct? Put in actuator manager class?
    double                                                   energy_level;
    double                                                   max_energy;
    double                                                   thresh_level;
    std::unique_ptr<fsm::depth0::stateless_foraging_fsm>     m_fsm;
    sensor::battery_sensor                                   bs;

}

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATELESS_FORAGING_CONTROLLER_HPP_ */
