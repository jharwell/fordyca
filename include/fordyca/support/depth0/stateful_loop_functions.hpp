/**
 * @file stateful_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/stateless_loop_functions.hpp"
#include "fordyca/support/depth0/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

class stateful_metrics_aggregator;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class stateful_loop_functions
 * @ingroup support depth0
 *
 * @brief Contains the simulation support functions for stateful foraging, such
 * as:
 *
 * - Sending robots their LOS each timestep
 * - Sending robots their position each timestep.
 * - Sending robot the current simulation tick each timestep.
 */
class stateful_loop_functions : public stateless_loop_functions,
                                public er::client<stateful_loop_functions> {
 public:
  stateful_loop_functions(void);
  ~stateful_loop_functions(void) override;

  void Init(ticpp::Element& node) override;
  void PreStep(void) override;
  void Reset(void) override;

 protected:
  void pre_step_final(void) override;

 private:
  using interactor =
      robot_arena_interactor<controller::depth0::stateful_controller>;

  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void controller_configure(controller::base_controller& c);

  // clang-format off
  std::unique_ptr<stateful_metrics_aggregator> m_metrics_agg;
  std::unique_ptr<interactor>                m_interactor{nullptr};
  // clang-format on
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_LOOP_FUNCTIONS_HPP_ */
