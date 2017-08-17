/**
 * @file loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_HPP_

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
#include "fordyca/support/block_distributor.hpp"
#include "fordyca/support/stat_collector.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/grid2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class loop_functions : public argos::CLoopFunctions {
 public:
  loop_functions();
  virtual ~loop_functions(void) {}

  virtual void Init(argos::TConfigurationNode& node);
  virtual void Reset();
  virtual void Destroy();
  virtual argos::CColor GetFloorColor(const argos::CVector2& plane_pos);
  virtual void PreStep();
  virtual bool IsExperimentFinished(void);
  virtual void PostExperiment(void) { std::exit(0); }

 private:
  loop_functions(const loop_functions& s) = delete;
  loop_functions& operator=(const loop_functions& s) = delete;
  int robot_on_block(const argos::CFootBotEntity& robot);

  argos::CFloorEntity* m_floor;
  std::unique_ptr<stat_collector> m_collector;
  std::shared_ptr<const struct logging_params> mc_logging_params;
  std::unique_ptr<const struct loop_functions_params> mc_loop_params;
  std::unique_ptr<block_distributor> m_distributor;
  std::shared_ptr<std::vector<representation::block>> m_blocks;
  std::unique_ptr<representation::grid2D> m_grid;
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_HPP_ */
