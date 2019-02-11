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
 class ee_decision_matrix;
 namespace state_machine = rcppsw::patterns::state_machine;
 namespace hal = rcppsw::robotics::hal;
 namespace utils = rcppsw::utils;
 namespace ta = rcppsw::task_allocation;
 namespace er = rcppsw::er;

 /*******************************************************************************
  * Class Definitions
  ******************************************************************************/
 /**
  * @class energy_subsystem
  * @ingroup controller
  *
  * @brief Handles the battery energy allocation and usage for each robot.
  * Determines how much energy will be allocated to the robot for the foraging task.
  *
  *
  *
  */
 class energy_subsystem : public er::client<energy_subsystem>{
  public:
    energy_subsystem(const params::energy_params* params, controller::saa_subsystem* saa);
    virtual ~energy_subsystem(void) = default;

    void run_fsm(void) {  e_fsm.run(); }

    void set_should_charge(bool set) { should_charge = set;  }
    bool get_should_charge(void) { return should_charge; }

    float desired_charge(void) { return ehigh_thres; }

    /**
     * @brief How the robot will update the energy thresholds and capacity for deciding
     * how much energy to allocate next time it starts foraging. It decides this based on
     * the performance from last round of foraging.
     */
    void energy_adapt(int k_robots);

    void set_task(ta::taskable* task) { e_fsm.set_taskable(task); }

    void success_pickup(void) { is_successful_pickup = 1; }

    void has_block(void) { e_fsm.got_block(); }
    void drop_block(void) { e_fsm.set_block(); }

  private:
    /**
     * @brief How the robot will perform and allocate or not allocate energy for foraging
     * when maximum battery potential has reached.
     */
    void endgame(int k_robots);


    float                                                 w[3];
    float                                                  wC[3];
    float                                                 elow_thres;
    float                                                 ehigh_thres;
    float                                                 capacity;
    std::string                                           EEE_method;
    std::shared_ptr<sensing_subsystem>                    m_sensing;
    int                                                   is_successful_pickup;
    int                                                   tau;
    int                                                   maxTau;
    bool                                                  is_new_thresh;
    bool                                                  is_EEE;
    controller::ee_decision_matrix*                       mc_matrix;
    bool                                                  should_charge;
    fsm::ee_max_fsm                                       e_fsm;
    float                                                 deltaE;

 };

 NS_END(fordyca, controller);

 #endif /* INCLUDE_FORDYCA_CONTROLLER_ENERGY_SUBSYSTEM_HPP_ */
