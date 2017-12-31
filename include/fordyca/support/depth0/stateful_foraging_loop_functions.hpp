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
NS_START(fordyca);

namespace robot_collectors = metrics::collectors::fsm;
namespace metrics { namespace collectors { namespace fsm {
class stateful_metrics_collector;
}}}
namespace controller { namespace depth0 { class foraging_controller; }}

NS_START(support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class stateful_foraging_loop_functions
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
  stateful_foraging_loop_functions(void);
  virtual ~stateful_foraging_loop_functions(void);

  void Init(argos::TConfigurationNode& node) override;
  void PreStep(void) override;
  void Destroy(void) override;
  void Reset(void) override;

 protected:
  robot_collectors::stateful_metrics_collector* stateful_collector(void) const;
  void pre_step_final(void) override;

 private:
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  stateful_foraging_loop_functions(const stateful_foraging_loop_functions& s) = delete;
  stateful_foraging_loop_functions& operator=(const stateful_foraging_loop_functions& s) = delete;

  std::unique_ptr<robot_collectors::stateful_metrics_collector> m_collector;
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_FORAGING_LOOP_FUNCTIONS_HPP_ */
