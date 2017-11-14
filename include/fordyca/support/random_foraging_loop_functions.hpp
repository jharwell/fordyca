/**
 * @file random_foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_RANDOM_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_RANDOM_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/diagnostics/block_stat_collector.hpp"
#include "fordyca/diagnostics/random_diagnostics_collector.hpp"
#include "fordyca/diagnostics/distance_diagnostics_collector.hpp"
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class random_foraging_loop_functions : public argos::CLoopFunctions,
                                       public rcppsw::common::er_client {
 public:
  random_foraging_loop_functions(void);
  virtual ~random_foraging_loop_functions(void) {}

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
  diagnostics::block_stat_collector* block_collector(void) const { return m_block_collector.get(); }
  diagnostics::distance_diagnostics_collector* distance_collector(void) const { return m_distance_collector.get(); }
  diagnostics::random_diagnostics_collector* random_collector(void) const { return m_random_collector.get(); }
  const argos::CRange<double>& nest_xrange(void) const { return m_nest_x; }
  const argos::CRange<double>& nest_yrange(void) const { return m_nest_y; }
  virtual void pre_step_final(void);

 private:
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  random_foraging_loop_functions(const random_foraging_loop_functions& s) = delete;
  random_foraging_loop_functions& operator=(const random_foraging_loop_functions& s) = delete;

  argos::CRange<double> m_nest_x;
  argos::CRange<double> m_nest_y;
  argos::CFloorEntity* m_floor;
  std::string m_sim_type;
  std::unique_ptr<diagnostics::random_diagnostics_collector> m_random_collector;
  std::unique_ptr<diagnostics::distance_diagnostics_collector> m_distance_collector;
  std::unique_ptr<diagnostics::block_stat_collector> m_block_collector;
  std::unique_ptr<representation::arena_map> m_map;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_RANDOM_FORAGING_LOOP_FUNCTIONS_HPP_ */
