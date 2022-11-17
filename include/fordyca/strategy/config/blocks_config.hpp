/**
 * \file blocks_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
  * Includes
******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/spatial/strategy/blocks/config/drop_config.hpp"
#include "cosm/spatial/strategy/explore/config/explore_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct blocks_config
  * \ingroup strategy config
  *
  * \brief Configuration for strategies related to things robots can do with/for
  * blocks when foraging.
  */
struct blocks_config final : public rconfig::base_config {
  cssexplore::config::explore_config explore{};
  cssblocks::config::drop_config drop{};
};

NS_END(config, strategy, fordyca);
