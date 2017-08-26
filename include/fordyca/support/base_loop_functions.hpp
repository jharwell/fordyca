/**
 * @file base_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_

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
#include "fordyca/support/stat_collector.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/params/loop_function_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class base_loop_functions : public argos::CLoopFunctions {
 public:
  base_loop_functions();
  virtual ~base_loop_functions(void) {}

  virtual void Init(argos::TConfigurationNode& node);
  virtual void Reset();
  virtual void Destroy();
  virtual argos::CColor GetFloorColor(const argos::CVector2& plane_pos);
  virtual void PreStep();
  virtual bool IsExperimentFinished(void);
  virtual void PostExperiment(void) { std::exit(0); }

 protected:
  void pre_step_iter(argos::CFootBotEntity& robot);
  void pre_step_final(void);
  int robot_on_block(const argos::CFootBotEntity& robot);
  int robot_id(const argos::CFootBotEntity& robot);
  representation::arena_map* map(void) const { return m_map.get(); }
  params::loop_function_repository* repo(void) const { return m_repo.get(); }

 private:
  base_loop_functions(const base_loop_functions& s) = delete;
  base_loop_functions& operator=(const base_loop_functions& s) = delete;

  argos::CRange<argos::Real> m_nest_x;
  argos::CRange<argos::Real> m_nest_y;
  argos::CFloorEntity* m_floor;
  std::unique_ptr<params::loop_function_repository> m_repo;
  std::unique_ptr<stat_collector> m_collector;
  std::unique_ptr<representation::arena_map> m_map;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_LOOP_FUNCTIONS_HPP_ */
