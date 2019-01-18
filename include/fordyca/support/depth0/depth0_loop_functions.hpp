/**
 * @file depth0_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "rcppsw/ds/type_map.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/depth0/controller_interactor_mapper.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace er = rcppsw::er;
namespace controller { namespace depth0 { class depth0_controller; }}
NS_START(support, depth0);

class depth0_metrics_aggregator;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
p * @class depth0_loop_functions
 * @ingroup support depth0
 *
 * @brief Contains the simulation support functions for depth0 foraging, such
 * as:
 *
 * - Sending robots their LOS each timestep
 * - Sending robots their position each timestep.
 * - Sending robot the current simulation tick each timestep.
 */
class depth0_loop_functions : public base_loop_functions,
                              public er::client<depth0_loop_functions> {
 public:
  depth0_loop_functions(void);
  ~depth0_loop_functions(void) override;

  void Init(ticpp::Element& node) override;
  void PreStep(void) override;
  void Reset(void) override;
  void Destroy(void) override;

 protected:
  virtual void pre_step_final(void);
  template<typename T>
  void set_robot_tick(argos::CFootBotEntity& robot) {
    auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    controller.tick(GetSpace().GetSimulationClock());
  }

 private:
  using interactor_map = rcppsw::ds::type_map<crw_itype,
                                              dpo_itype,
                                              mdpo_itype>;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  void controller_configure(controller::base_controller* c);

  /* clang-format off */
  std::unique_ptr<depth0_metrics_aggregator> m_metrics_agg;
  std::unique_ptr<interactor_map>            m_interactors;
  /* clang-format on */
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_LOOP_FUNCTIONS_HPP_ */
