/**
 * \file env_dynamics_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/config/base_env_dynamics_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_dynamics_config
 * \ingroup argos support tv config
 *
 * \brief Configuration for the \ref env_dynamics.
 */
struct env_dynamics_config final : public ctv::config::base_env_dynamics_config {
  ctv::config::temporal_penalty_config cache_usage_penalty{};
};

NS_END(tv, config, support, fordyca, argos);

