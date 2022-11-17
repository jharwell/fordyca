/**
 * \file dpo_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/subsystem/perception/config/pheromone_config.hpp"
#include "cosm/subsystem/perception/config/rlos_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct dpo_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for the Decaying Pheromone Object (DPO) perception
 * subsystem.
 */
struct dpo_config final : public rconfig::base_config {
  cspconfig::pheromone_config pheromone {};
  cspconfig::rlos_config rlos {};
};

NS_END(config, perception, subsystem, fordyca);

