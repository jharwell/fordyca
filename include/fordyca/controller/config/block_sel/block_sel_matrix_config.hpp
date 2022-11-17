/**
 * \file block_sel_matrix_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/config/base_config.hpp"

#include "fordyca/controller/config/block_sel/block_priority_config.hpp"
#include "fordyca/controller/config/block_sel/block_pickup_policy_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_sel_matrix_config
 * \ingroup controller config block_sel
 *
 * \brief XML parameters for the \ref block_sel_matrix
 */
struct block_sel_matrix_config final : public rconfig::base_config {
  block_priority_config priorities {};
  block_pickup_policy_config pickup_policy{};
};

NS_END(block_sel, config, controller, fordyca);

