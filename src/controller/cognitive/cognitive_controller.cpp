/**
 * \file cognitive_controller.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/cognitive_controller.hpp"

#include "cosm/ds/cell2D.hpp"

#include "fordyca/subsystem/perception/config/perception_config.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cognitive_controller::cognitive_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.cognitive"),
      m_perception() {}

cognitive_controller::~cognitive_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cognitive_controller::perception(
    std::unique_ptr<fsperception::foraging_perception_subsystem> perception) {
  m_perception = std::move(perception);
}

double cognitive_controller::los_dim(void) const {
  return perception()->los_dim();
} /* los_dim() */

void cognitive_controller::reset(void) {
  foraging_controller::reset();
  m_perception->reset();
} /* reset() */

NS_END(cognitive, controller, fordyca);
