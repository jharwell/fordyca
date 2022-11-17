/**
 * \file block_priority_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_priority_config
 * \ingroup controller config block_sel
 *
 * \brief Configuration for setting the block priorities of different block
 * types.
 */
struct block_priority_config final : public rconfig::base_config {
  double cube{-1.0};
  double ramp{-1.0};
};

NS_END(block_sel, config, controller, fordyca);

