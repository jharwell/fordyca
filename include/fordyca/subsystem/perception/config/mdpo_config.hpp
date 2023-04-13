/**
 * \file mdpo_config.hpp
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
#include "cosm/subsystem/perception/rlos/config/rlos_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct mdpo_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for the Mapped Decaying Pheromone Object (MDPO)
 * perception subsystem.
 */
struct mdpo_config final : public rconfig::base_config {
  cspconfig::pheromone_config pheromone {};
  csperception::rlos::config::rlos_config rlos {};
};

NS_END(config, perception, subsystem, fordyca);

