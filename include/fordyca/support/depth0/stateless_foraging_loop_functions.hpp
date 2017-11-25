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
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace metrics { namespace collectors {
class block_metrics_collector;
namespace robot_metrics {
class stateless_metrics_collector;
class distance_metrics_collector;
}
}}

NS_START(support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class stateless_foraging_loop_functions : public argos::CLoopFunctions,
                                       public rcppsw::er::client {
 public:
  stateless_foraging_loop_functions(void);
  virtual ~stateless_foraging_loop_functions(void);

  void Init(argos::TConfigurationNode& node) override;
  void Reset() override;
  void Destroy() override;
  void PreStep() override;
  bool IsExperimentFinished(void) override;
  void PostExperiment(void) override;

 protected:
  template<typename T>
  void set_robot_pos(argos::CFootBotEntity& robot) {
    argos::CVector2 pos;
    pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
            const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    T& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
    controller.robot_loc(pos);
  } /* set_robot_los() */

  /**
   * @brief Check if a robot is on top of a block. If, so return the block index.
   *
   * @param robot The robot to check
   *
   * @return The block index, or -1 if the robot is not on top of a block.
   */
  int robot_on_block(const argos::CFootBotEntity& robot);

  /**
   * @brief Get the ID of the robot as an integer.
   */
  int robot_id(const argos::CFootBotEntity& robot);
  representation::arena_map* map(void) const { return m_map.get(); }
  argos::CFloorEntity* floor(void) const { return m_floor; }

  metrics::collectors::block_metrics_collector* block_collector(void) const;
  metrics::collectors::robot_metrics::distance_metrics_collector* distance_collector(void) const;
  metrics::collectors::robot_metrics::stateless_metrics_collector* stateless_collector(void) const;

  const argos::CRange<double>& nest_xrange(void) const { return m_nest_x; }
  const argos::CRange<double>& nest_yrange(void) const { return m_nest_y; }
  virtual void pre_step_final(void);

 private:
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  stateless_foraging_loop_functions(const stateless_foraging_loop_functions& s) = delete;
  stateless_foraging_loop_functions& operator=(const stateless_foraging_loop_functions& s) = delete;

  argos::CRange<double> m_nest_x;
  argos::CRange<double> m_nest_y;
  argos::CFloorEntity* m_floor;
  std::string m_sim_type;
  std::unique_ptr<metrics::collectors::robot_metrics::stateless_metrics_collector> m_stateless_collector;
  std::unique_ptr<metrics::collectors::robot_metrics::distance_metrics_collector> m_distance_collector;
  std::unique_ptr<metrics::collectors::block_metrics_collector> m_block_collector;
  std::unique_ptr<representation::arena_map> m_map;
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATELESS_FORAGING_LOOP_FUNCTIONS_HPP_ */
