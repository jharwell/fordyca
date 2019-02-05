/**
 * @file actuation_subsystem.hpp
 *
 * @copyright 2019 Anthony Chen/John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_ENERGY_SUBSYSTEM_HPP
#define INCLUDE_FORDYCA_CONTROLLER_ENERGY_SUBSYSTEM_HPP

/*******************************************************************************
 * Includes
 ******************************************************************************/
 #include "fordyca/params/energy_params.hpp"
 #include "fordyca/fsm/ee_max_fsm.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, controller);

 namespace state_machine = rcppsw::patterns::state_machine;
 namespace hal = rcppsw::robotics::hal;
 namespace utils = rcppsw::utils;
 namespace ta = rcppsw:task_allocation;

 /*******************************************************************************
  * Class Definitions
  ******************************************************************************/
 /**
  * @class energy_subsystem
  * @ingroup controller
  *
  * @brief Handles the battery energy allocation and usage for each robot.
  * Determines how much energy will be allocated to
  *
  *
  */
 class energy_subsystem {
  public:
    energy_subsystem(const params::energy_params* params, const ta::taskable* task,
                     controller::saa_subsystem* saa);
    virtual ~energy_subsystem(void) = default;

    virtual void reset(void) {}
    void success_pickup(int success) { is_successful_pickup = 1;}

    void update(void);

  private:

    void energy_check(void);
    void endgame(void);
    void energy_drain(void);
    void energy_adapt(int k_robots, int f_success);

    const controller::ee_decision_matrix* const           mc_matrix;
    std::unique_ptr<fsm::ee_max_fsm>                      e_fsm;
    std::shared_ptr<sensing_subsystem>                    m_sensing;
    int                                                   w[3];
    int                                                   wC[3];
    float                                                 deltaE;
    float                                                 elow_thres;
    float                                                 ehigh_thres;
    float                                                 capacity;
    int                                                   is_successful_pickup;
    bool                                                  is_new_thresh;
    bool                                                  is_EEE;
    int                                                   tau;
    int                                                   maxTau;
    std::string                                           EEE_method;

 }
