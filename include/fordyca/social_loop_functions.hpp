/**
 * @file social_loop_functions.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
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

#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

/*******************************************************************************
 * Classes
 ******************************************************************************/
class social_loop_functions : public argos::CLoopFunctions {

 public:

  social_loop_functions();
  virtual ~social_loop_functions() {}

  virtual void Init(argos::TConfigurationNode& t_tree);
  virtual void Reset();
  virtual void Destroy();
  virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
  virtual void PreStep();

 private:

  argos::Real m_fFoodSquareRadius;
  argos::CRange<argos::Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
  std::vector<argos::CVector2> m_cFoodPos;
  argos::CFloorEntity* m_pcFloor;
  argos::CRandom::CRNG* m_pcRNG;

  std::string m_strOutput;
  std::ofstream m_cOutput;

  uint m_unCollectedFood;
  int m_nEnergy;
  uint m_unEnergyPerFoodItem;
  uint m_unEnergyPerWalkingRobot;
};

#endif
