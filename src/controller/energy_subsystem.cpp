/**
 * @file energy_subsystem.cpp
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
#include "fordyca/controller/energy_subsystem.hpp"
#include "fordyca/params/perception/energy_params.hpp"

#include <algorithm>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace energy_fsm = fsm::depth0::ee_max_fsm;

NS_START(controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
energy_subsystem::energy_subsystem(
    const struct params::energy_params* const params, ta:taskable* task, controller::saa_subsystem* saa)
    : ER_CLIENT_INIT("fordyca.controller.energy"),
      w{params->weight1, params->weight2, params->weight3},
      wC{params->weight1C, params->weight2C, params->weight3C},
      elow_thres(params->elow),
      ehigh_thres(params->ehigh),
      capacity(params->capacity),
      EEE_method(params->EEE),
      m_sensing(saa->sensing()),
      is_successful_pickup(0),
      tau(10),
      maxTau(10),
      is_new_thresh(true),
      is_EEE(false),
      mc_matrix(),
      e_fsm(task, mc_matrix, saa) {}

energy_subsystem::~energy_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void energy_subsystem::reset(void) {  }

void energy_check(void)  { }

void energy_subsystem::endgame(int k_robots) {
  if(EEE_method == "Well-informed") {
      if(tau > 0) {
        tau = tau - 1;
      } else {
        m_sensing->battery()->setCharge(ehigh_thres);
        maxTau = maxTau + 10;
        tau = maxTau;
      }
  } else if(EEE_method == "Ill-informed") {
      if(tau > 0) {
        tau = tau - 1;
      } else {
        m_sensing->battery()->setCharge(ehigh_thres);
        maxTau = maxTau + 10;
        tau = maxTau;
        elow_thres = elow_thres - (is_successful_pickup * max(0, (energy_init - deltaE))*w1)
                                + ((!is_successful_pickup)*w2) + (k_robots*w3);
        mc_matrix->setData(elow_thres, ehigh_thres);
      }
  } else if(EEE_method == "Null-informed") {
      m_sensing->battery()->setCharge(0.0);
  }
}


void energy_subsystem::energy_adapt(int k_robots) {
  if(is_EEE) {
    endgame(k_robots);
  } else {

    if(e_fsm->current_state() == energy_fsm::ST_CHARGING && is_new_thresh) {
      elow_thres = elow_thres - (is_successful_pickup * max(0, (energy_init - deltaE))*w1)
                              + ((!is_successful_pickup)*w2) + (k_robots*w3);

      capacity = capacity - (is_successful_pickup * max(0, (energy_init - deltaE))*w1C)
                          + ((!is_successful_pickup)*w2C) + (k_robots*w3C);

      if(elow_thres < 0)
        elow_thres = 0;

      ehigh_thres = elow_thres + capacity;

      if(ehigh_thres > 1)
        ehigh_thres = 1;

      if((ehigh_thres - elow_thres) == 1) {
        is_EEE = true;
      }

      m_sensing->battery()->setCharge(ehigh_thres);

      mc_matrix->setData(elow_thres, ehigh_thres);

      is_successful_pickup = 0;
      is_new_thresh = false;
    } else if (e_fsm->current_state() == energy_fsm::ST_FORAGING) {
      is_new_thresh = true;
    }
  }
}

NS_END(controller, fordyca);
