/**
 * \file tv_manager_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/tv/config/population_dynamics_config.hpp"

#include "fordyca/argos/support/tv/config/env_dynamics_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct tv_manager_config
 * \ingroup argos support tv config
 *
 * \brief Configuration for the \ref tv_manager.
 */
struct tv_manager_config final : public rconfig::base_config {
  env_dynamics_config env_dynamics{};
  ctv::config::population_dynamics_config population_dynamics{};
};

NS_END(config, tv, support, argos, fordyca);

