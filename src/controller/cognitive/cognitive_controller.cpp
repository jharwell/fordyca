/**
 * \file cognitive_controller.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    : ER_CLIENT_INIT("fordyca.controller.cognitive"), m_perception() {}

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
