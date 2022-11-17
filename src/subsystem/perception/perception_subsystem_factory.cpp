/**
 * \file perception_subsystem_factory.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/perception_subsystem_factory.hpp"

#include "cosm/ds/cell2D.hpp"

#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, subsystem, perception);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
perception_subsystem_factory::perception_subsystem_factory(void) {
  register_type<dpo_perception_subsystem>(kDPO);
  register_type<mdpo_perception_subsystem>(kMDPO);
}

NS_END(perception, subsystem, fordyca);
