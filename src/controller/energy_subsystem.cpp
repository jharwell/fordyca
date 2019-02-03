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
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
energy_subsystem::energy_subsystem(
    const struct params::energy_params* const params, ta:taskable* task)
    : ER_CLIENT_INIT("fordyca.controller.energy"),
      w{params->weight1, params->weight2, params->weight3},
      wC{params->weight1C, params->weight2C, params->weight3C},
      elow_thres(params->elow),
      ehigh_thres(params->ehigh),
      capacity(params->capacity),
      EEE_method(params->EEE),
      e_fsm(task) {}

energy_subsystem::~energy_subsystem(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void energy_subsystem::reset(void) {  }

void energy_check(void)  { }

void energy_subsystem::endgame(void) { }

void energy_subsystem::energy_drain(void) {
/*
  switch(/*state*) {
    case searching:
      energy = energy - alphaS;
      deltaE = deltaE + alphaS;
      break;
    case retreating:
      energy = energy - alphaR;
      deltaE = deltaE + alphaR;
      break;
    case collecting:
      energy = energy - p;
      deltaE = deltaE + p;
      break;
  }
  */
}

void energy_subsystem::energy_adapt(int k_robots, int f_success) {
  elow_thres = elow_thres - (f_success * max(0, (energy_init - deltaE))*w1)
                          + ((!f_success)*w2) + (k_robots*w3);

  capacity = capacity - (f_success * max(0, (energy_init - deltaE))*w1C)
                      + ((!f_success)*w2C) + (k_robots*w3C);

  ehigh_thres = elow_thres + capacity;
}

NS_END(controller, fordyca);
