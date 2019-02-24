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
#include "fordyca/controller/ee_decision_matrix.hpp"
#include "fordyca/fsm/ee_max_fsm.hpp"
#include "fordyca/params/energy_params.hpp"

#include <algorithm>
#include <stdio.h>      /*  NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm {class ee_max_fsm; }

NS_START(controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
energy_subsystem::energy_subsystem(
    const struct params::energy_params* const params, controller::saa_subsystem* saa)
    : ER_CLIENT_INIT("fordyca.controller.energy"),
      w{params->weight1, params->weight2, params->weight3},
      wC{params->weight1C, params->weight2C, params->weight3C},
      elow_thres(params->elow),
      ehigh_thres(params->ehigh),
      capacity(params->capacity),
      EEE_method(params->EEE),
      m_sensing(saa->sensing()),
      is_successful_pickup(0),
      tau(0),
      maxTau(100),
      is_new_thresh(true),
      is_EEE(false),
      mc_matrix(new controller::ee_decision_matrix(params)),
      should_charge(false),
      activate(params->activated),
      isLabella(params->labella),
      succ(0),
      fail(0),
      is_new_labella(true),
      P1(0.033),
      e_fsm(mc_matrix, saa) {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/

void energy_subsystem::endgame(int k_robots) {
  if(EEE_method == "Well-informed") {
      ER_INFO("ENTERED WELL-INFORMED");
      set_should_charge(true);
      maxTau = maxTau + 50;
      tau = maxTau;
  } else if(EEE_method == "Ill-informed") {
      ER_INFO("ENTERED ILL-INFORMED");
      deltaE = ehigh_thres - m_sensing->battery().readings().available_charge;
      double remaining = ehigh_thres - deltaE;
      ER_INFO("SUBSYSTEM:\tRemaining: %f", remaining);
      elow_thres = elow_thres - (std::max(0.0, (remaining))*(w[0]))
                              + ((!is_successful_pickup)*(w[1])) + (k_robots*(w[2]));
      if(elow_thres < 0)
        elow_thres = 0;

      if(elow_thres >= 1.0)
        set_should_charge(false);
      else {
        set_should_charge(true);
      }
      maxTau = maxTau + 50;
      tau = maxTau;
      mc_matrix->setData(elow_thres, ehigh_thres);
  } else if(EEE_method == "Null-informed") {
      ER_INFO("ENTERED NULL-INFORMED");
      set_should_charge(false);
  }

  ER_INFO("SUBSYSTEM:\tSuccessful Pickup: %d", is_successful_pickup);
  ER_INFO("SUBSYSTEM:\tRobots: %d", k_robots);
  ER_INFO("SUBSYSTEM:\tLower Energy Threshold: %f", elow_thres);
  ER_INFO("SUBSYSTEM:\tUpper Energy Threshold: %f", ehigh_thres);
  ER_INFO("SUBSYSTEM:\tCapacity: %f", capacity);

  is_successful_pickup = 0;
}


bool energy_subsystem::labella(void) {
  ER_INFO("isLabella:\t %d", isLabella);
  ER_INFO("is_new_labella:\t %d", is_new_labella);
  ER_INFO("is_successful_pickup:\t %d", is_successful_pickup);
  if(isLabella) {
    if(is_new_labella) {
      if(is_successful_pickup == 1) {
        fail = 0;
        succ = succ + 1;
        P1 = P1 + (succ*0.005);
        if (P1 > 0.05) {
          P1 = 0.05;
        }
      } else {
        fail = fail + 1;
        succ = 0;
        P1 = P1 - (fail*0.005);
        if (P1 < 0.0015) {
          P1 = 0.0015;
        }
      }
      is_new_labella = false;
    }

    srand(time(NULL));

    double guess = (double) rand()/ RAND_MAX;
    ER_INFO("guess:\t %f", guess);
    if (guess <= P1) {
      return true;
    } else {
      return false;
    }

  } else {
    return true;
  }

  ER_INFO("P1:\t %f", P1);
  ER_INFO("fail:\t %d", fail);
  ER_INFO("succ:\t %d", succ);
}


void energy_subsystem::energy_adapt(int k_robots) {
  ER_INFO("TAU:\t\t%d", tau);
  if(e_fsm.current_state() == fsm::ee_max_fsm::ST_CHARGING && is_new_thresh) {
    if(tau < maxTau) {
      tau = tau + 1;
    } else {
      // if labella
      if(labella()) {
        if(activate) {
          if(is_EEE) {
            ER_INFO("SUBSYSTEM:\tRobot is entering EEE: %s", EEE_method);
            endgame(k_robots);
          } else {
            /* updated based on three criteria:
                How early if achieved a successful pickup
                If there was a failed pickup
                If encountered any robots.
            */
            ER_INFO("SUBSYSTEM 0:\tLower Energy Threshold: %f", elow_thres);
            ER_INFO("SUBSYSTEM 0:\tUpper Energy Threshold: %f", ehigh_thres);
            ER_INFO("SUBSYSTEM 0:\tCapacity: %f", capacity);
            deltaE = ehigh_thres - m_sensing->battery().readings().available_charge;
            double remaining = ehigh_thres - deltaE;
            ER_INFO("SUBSYSTEM:\tSuccessful Pickup: %d", is_successful_pickup);
            elow_thres = elow_thres - (std::max(0.0, (remaining))*(w[0]))
                                    + ((!is_successful_pickup)*(w[1])) + (k_robots*(w[2]));

            capacity = capacity - (is_successful_pickup * std::max(0.0, (remaining))*(wC[0]))
                                + ((!is_successful_pickup)*(wC[1])) + (k_robots*(wC[2]));

            ER_INFO("SUBSYSTEM 1:\tLower Energy Threshold: %f", elow_thres);
            ER_INFO("SUBSYSTEM 1:\tCapacity: %f", capacity);

            if(elow_thres < 0)
              elow_thres = 0.1;

            ehigh_thres = elow_thres + capacity;

            if(ehigh_thres > 1)
              ehigh_thres = 1;

            if(capacity > 1)
              capacity = 1;

            if(elow_thres > ehigh_thres)
              elow_thres = ehigh_thres - capacity;

            if(ehigh_thres == 1) {
              is_EEE = true;
            }

            set_should_charge(true);

            ER_INFO("SUBSYSTEM:\tEnergy Level: %f", m_sensing->battery().readings().available_charge);

            mc_matrix->setData(elow_thres, ehigh_thres);

          }
        } else {
          ehigh_thres = 1.0;
          elow_thres = 0.0;
          capacity = 1.0;
          deltaE = ehigh_thres - m_sensing->battery().readings().available_charge;

          set_should_charge(true);

          mc_matrix->setData(elow_thres, ehigh_thres);

        }
        is_successful_pickup = 0;
        is_new_thresh = false;
        tau = 0;
      }
    }
  } else if (e_fsm.current_state() == fsm::ee_max_fsm::ST_FORAGING) {
    is_new_thresh = true;
    is_new_labella = true;
  }

}

NS_END(controller, fordyca);
