/**
 * @file foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include "fordyca/support/random_foraging_loop_functions.hpp"
#include "fordyca/diagnostics/random_diagnostics_collector.hpp"
#include "fordyca/diagnostics/depth0/collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller { namespace depth0 { class foraging_controller; }}
NS_START(support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class foraging_loop_functions : public random_foraging_loop_functions {
 public:
  foraging_loop_functions() : m_random_collector(), m_depth0_collector() {}
  virtual ~foraging_loop_functions(void) {}

  void Init(argos::TConfigurationNode& node) override;
  void PreStep(void) override;
  void Destroy(void) override;
  void Reset(void) override;

 protected:
  void set_robot_los(argos::CFootBotEntity& robot);
  void set_robot_tick(argos::CFootBotEntity& robot);
  void handle_nest_block_drop(controller::depth0::foraging_controller& controller);
  void handle_free_block_pickup(argos::CFootBotEntity& robot);

 private:
  void pre_step_final(void);
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  foraging_loop_functions(const foraging_loop_functions& s) = delete;
  foraging_loop_functions& operator=(const foraging_loop_functions& s) = delete;

  std::unique_ptr<diagnostics::random_diagnostics_collector> m_random_collector;
  std::unique_ptr<diagnostics::depth0::collector> m_depth0_collector;
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_FORAGING_LOOP_FUNCTIONS_HPP_ */
