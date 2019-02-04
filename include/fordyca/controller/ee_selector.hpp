/**
 * @file energy_selector.hpp
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

 #ifndef INCLUDE_FORDYCA_CONTROLLER_ENERGY_SELECTOR_HPP_
 #define INCLUDE_FORDYCA_CONTROLLER_ENERGY_SELECTOR_HPP_

 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include <list>

 #include "fordyca/controller/ee_decision_matrix.hpp"
 #include "rcppsw/er/client.hpp"
 #include "rcppsw/math/vector2.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, controller);
 namespace rmath = rcppsw::math;

 /*******************************************************************************
  * Class Definitions
  ******************************************************************************/
 /**
  * @class energy_selector
  * @ingroup controller depth0
  *
  * @brief Select the energy thresholds and capacity that a robot knows about,
  * for use in energy FSM.
  */
 class ee_selector : public rcppsw::er::client<ee_selector> {
 public:
  explicit energy_selector(const ee_decision_matrix* sel_matrix);

  ~energy_selector(void) override = default;

  energy_selector& operator=(const energy_selector& other) = delete;
  energy_selector(const energy_selector& other) = delete;

  float getLowerThres(void);
  float getHigherThres(void);

 private:

   // maybe a function that checks that the thresholds are positive and one is
   // greater than the other.

  /* clang-format off */
  const ee_decision_matrix* const mc_matrix;
  /* clang-format on */
};
