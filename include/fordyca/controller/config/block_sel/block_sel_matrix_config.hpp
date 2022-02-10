/**
 * \file block_sel_matrix_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
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

