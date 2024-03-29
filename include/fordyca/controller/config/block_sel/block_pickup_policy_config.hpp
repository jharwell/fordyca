/**
 * \file block_pickup_policy_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
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

