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
#include <vector>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/diagnostics/block_stat_collector.hpp"
#include "fordyca/diagnostics/base_robot_stat_collector.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/params/loop_function_repository.hpp"

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
  virtual void pre_step_iter(argos::CFootBotEntity& robot);
  virtual void pre_step_final(void);

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
  params::loop_function_repository* repo(void) const { return m_repo.get(); }
  argos::CFloorEntity* floor(void) const { return m_floor; }
  diagnostics::base_robot_stat_collector* robot_collector(void) const { return m_robot_collector.get(); }
  diagnostics::block_stat_collector* block_collector(void) const { return m_block_collector.get(); }
  const argos::CRange<double>& nest_xrange(void) const { return m_nest_x; }
  const argos::CRange<double>& nest_yrange(void) const { return m_nest_y; }

 private:
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;
  random_foraging_loop_functions(const random_foraging_loop_functions& s) = delete;
  random_foraging_loop_functions& operator=(const random_foraging_loop_functions& s) = delete;

  argos::CRange<double> m_nest_x;
  argos::CRange<double> m_nest_y;
  argos::CFloorEntity* m_floor;
  std::string m_sim_type;
  std::unique_ptr<params::loop_function_repository> m_repo;
  std::unique_ptr<diagnostics::base_robot_stat_collector> m_robot_collector;
  std::unique_ptr<diagnostics::block_stat_collector> m_block_collector;
  std::unique_ptr<representation::arena_map> m_map;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_RANDOM_FORAGING_LOOP_FUNCTIONS_HPP_ */
