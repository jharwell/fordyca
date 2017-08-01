/**
 * @file social_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SOCIAL_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SOCIAL_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/parameter_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Classes
 ******************************************************************************/
class social_loop_functions : public argos::CLoopFunctions {
 public:
  social_loop_functions();
  virtual ~social_loop_functions(void) {}

  virtual void Init(argos::TConfigurationNode& t_tree);
  virtual void Reset();
  virtual void Destroy();
  virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
  virtual void PreStep();

 private:
  social_loop_functions(const social_loop_functions& s) = delete;
  social_loop_functions& operator=(const social_loop_functions& s) = delete;

  argos::CRange<argos::Real> m_arena_x;
  argos::CRange<argos::Real> m_arena_y;
  std::vector<argos::CVector2> m_food_pos;
  argos::CFloorEntity* m_floor;
  argos::CRandom::CRNG* m_rng;

  std::string m_ofname;
  std::ofstream m_ofile;

  uint m_uncollected_food;
  int m_energy;
  uint m_energy_per_moving_robot;
  struct food_params m_food_params;
  parameter_manager m_param_manager;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SOCIAL_LOOP_FUNCTIONS_HPP_ */
