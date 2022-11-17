/**
 * \file block_pickup_policy_config.hpp
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

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_pickup_policy_config
 * \ingroup controller config block_sel
 *
 * \brief Configuration for robot block pickup policies.
 */
struct block_pickup_policy_config final : public rconfig::base_config {
  std::string policy{};
  double      prox_dist{0};
};

NS_END(block_sel, config, controller, fordyca);

