/**
 * \file caches_config.hpp
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
  * \struct caches_config
  * \ingroup strategy config
  *
  * \brief Configuration for strategies related to things robots can do with/for
  * caches when foraging.
  */
struct caches_config final : public rconfig::base_config {
  cssexplore::config::explore_config explore{};
};

NS_END(config, strategy, fordyca);
