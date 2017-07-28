/**
 * @file sensor_manager.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/sensor_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constant Definitions
 ******************************************************************************/

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Forward Declarations
 ******************************************************************************/

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool sensor_manager::calc_diffusion_vector(argos::CVector2& vector) {
  /* Get readings from proximity sensor */
  const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = proximity_->GetReadings();
  /* Sum them together */
  argos::CVector2 ccalc_diffusion_vector;
  for(size_t i = 0; i < tProxReads.size(); ++i) {
    vector += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }
  /*
   * If the angle of the vector is small enough and the closest obstacle is far
   * enough, ignore the vector and go straight, otherwise return it
   */
  if(m_sDiffusionParams.go_straight_angle_range.WithinMinBoundIncludedMaxBoundIncluded(ccalc_diffusion_vector.Angle()) &&
     ccalc_diffusion_vector.Length() < m_sDiffusionParams.Delta ) {
    return false;
  }
  return true;
}

NS_END(fordyca);
