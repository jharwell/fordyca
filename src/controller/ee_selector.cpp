/**
 * @file energy_selector.cpp
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


 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include "fordyca/controller/energy_selector.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, controller);
 using edecm = ee_decision_matrix;

 /*******************************************************************************
  * Constructors/Destructor
  ******************************************************************************/
 energy_selector::energy_selector(const ee_decision_matrix* const sel_matrix)
     : ER_CLIENT_INIT("fordyca.controller.depth0.energy_selector"),
       mc_matrix(sel_matrix) {}

/*******************************************************************************
 * Member Functions
******************************************************************************/
float energy_selector::getLowerThres(void) {
  return mc_matrix->e_lowerT;
}

float energy_selector::getHigherThres(void) {
  return mc_matrix->e_higherT;
}

NS_END(controller, fordyca);
