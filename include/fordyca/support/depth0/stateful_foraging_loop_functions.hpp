/**
 * @file stateful_foraging_loop_functions.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/stateless_foraging_loop_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class stateful_foraging_loop_functions
 * @ingroup support depth0
 *
 * @brief Contains the simulation support functions for stateful foraging, such
 * as:
 *
 * - Sending robots their LOS each timestep
 * - Sending robots their position each timestep.
 * - Sending robot the current simulation tick each timestep.
 */
class stateful_foraging_loop_functions : public stateless_foraging_loop_functions {
 public:
  stateful_foraging_loop_functions(void) = default;
  ~stateful_foraging_loop_functions(void) override = default;

  void Init(argos::TConfigurationNode& node) override;
  void PreStep(void) override;

 private:
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_FORAGING_LOOP_FUNCTIONS_HPP_ */
