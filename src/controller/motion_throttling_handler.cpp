/**
 * @file motion_throttling_handler.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/motion_throttling_handler.hpp"
#include "rcppsw/control/waveform_params.hpp"
#include "rcppsw/control/waveform_generator.hpp"
#include "rcppsw/control/waveform.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
motion_throttling_handler::motion_throttling_handler(
    const ct::waveform_params * const params)
    : m_waveform(ct::waveform_generator()(params->type, params)) {
}

motion_throttling_handler::~motion_throttling_handler(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void motion_throttling_handler::update(uint timestep) {
  if (m_en) {
    m_current = m_waveform->value(timestep);
  } else {
    m_current = 0;
  }
} /* update() */

NS_END(controller, fordyca);
