/**
 * \file strategy_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
