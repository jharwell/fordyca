/**
 * @file stateless_foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/common/common.hpp"
#include "fordyca/support/base_foraging_loop_functions.hpp"
#include "fordyca/support/depth0/arena_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
struct output_params;
class loop_function_repository;
namespace arena { struct arena_map_params; }
}

NS_START(support, depth0);
class stateless_metrics_aggregator;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class statless_foraging_loop_functions
 * @ingroup support depth0
 *
 * @brief Contains the simulation support functions for stateless foraging:
 *
 * - Sending robots block pickup/block drop signals if they are waiting for
 *   them.
 * - Handling block distribution.
 */
class stateless_foraging_loop_functions : public base_foraging_loop_functions,
                                          public er::client<stateless_foraging_loop_functions>  {
 public:
  stateless_foraging_loop_functions(void);
  ~stateless_foraging_loop_functions(void) override;

  void Init(ticpp::Element& node) override;
  void Reset() override;
  void Destroy() override;
  void PreStep() override;

 protected:
  const representation::arena_map* arena_map(void) const { return m_arena_map.get(); }
  representation::arena_map* arena_map(void) { return m_arena_map.get(); }
  const std::string& output_root(void) const { return m_output_root; }

  virtual void pre_step_final(void);

  template<typename T>
  void set_robot_tick(argos::CFootBotEntity& robot) {
    auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    controller.tick(GetSpace().GetSimulationClock());
  }

  void ndc_push(void) {
    ER_NDC_PUSH("[t=" + std::to_string(GetSpace().GetSimulationClock()) + "]");
  }
  void ndc_pop(void) { ER_NDC_POP(); }
 private:
  using interactor =
      arena_interactor<controller::depth0::stateless_foraging_controller>;

  void arena_map_init(params::loop_function_repository& repo);
  void output_init(const struct params::arena::arena_map_params* const arena,
                   struct params::output_params* const output);
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  // clang-format off
  std::string                                   m_output_root{""};
  std::unique_ptr<stateless_metrics_aggregator> m_metrics_agg{nullptr};
  std::unique_ptr<representation::arena_map>    m_arena_map;
  std::unique_ptr<interactor>                   m_interactor;
  // clang-format on
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_FORAGING_LOOP_FUNCTIONS_HPP_ */
