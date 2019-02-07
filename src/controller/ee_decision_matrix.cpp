/**
 * @file ee_decision_matrix.cpp
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
 #include "fordyca/controller/ee_decision_matrix.hpp"
 // Insert Params files?

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, controller);

 /*******************************************************************************
  * Constructors/Destructors
  ******************************************************************************/
  ee_decision_matrix::ee_decision_matrix()
      : e_lowerT(0.0),
        e_higherT(0.0) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
  void ee_decision_matrix::setData(float eLow, float eHigh) {
    e_lowerT = eLow;
    e_higherT = eHigh;
  }


 NS_END(controller, fordyca);
