/**
 * \file cache_pickup_policy_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct cache_pickup_policy_config
 * \ingroup controller config cache_sel
 *
 * \brief Configuration for robot cache pickup policies.
 */
struct cache_pickup_policy_config final : public rconfig::base_config {
  std::string      policy{};
  rtypes::timestep timestep{0};
  size_t           cache_count{0};
  size_t           cache_size{0};
};

NS_END(cache_sel, config, controller, fordyca);

