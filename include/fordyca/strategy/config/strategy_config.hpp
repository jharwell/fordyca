/**
 * \file strategy_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
  * Includes
******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "fordyca/strategy/config/blocks_config.hpp"
#include "fordyca/strategy/config/caches_config.hpp"
#include "fordyca/strategy/config/nest_config.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct strategy_config
  * \ingroup strategy config
  *
  * \brief Configuration for different strategies that can be employed by
  * robots for doing things like exploring, collision avoidance, etc.
  */
struct strategy_config final : public rconfig::base_config {
  /* clang-format off */
  nest_config   nest{};
  blocks_config blocks{};
  caches_config caches{};
  /* clang-format on */
};

NS_END(config, strategy, fordyca);
