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
#include "fordyca/support/tv/motion_throttling_handler.hpp"
#include "rcppsw/control/config/waveform_config.hpp"
#include "rcppsw/control/waveform.hpp"
#include "rcppsw/control/waveform_generator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
motion_throttling_handler::motion_throttling_handler(
    const rct::config::waveform_config* const config)
    : m_waveform(rct::waveform_generator()(config->type, config)) {}

motion_throttling_handler::~motion_throttling_handler(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void motion_throttling_handler::update(uint timestep) {
  if (m_en) {
    m_active = m_waveform->value(timestep);
  } else {
    m_active = 0;
  }
  m_applied = m_waveform->value(timestep);
} /* update() */

NS_END(tv, support, fordyca);
