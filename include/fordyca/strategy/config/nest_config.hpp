/**
 * \file nest_config.hpp
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
#include "rcppsw/config/base_config.hpp"

#include "cosm/spatial/strategy/nest/config/acq_config.hpp"
#include "cosm/spatial/strategy/nest/config/exit_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct nest_config
  * \ingroup strategy config
  *
  * \brief Configuration for strategies related to things robots can do related
  * to the nest while foraging.
  */
struct nest_config final : public rconfig::base_config {
  cssnest::config::acq_config acq{};
  cssnest::config::exit_config exit{};
};

NS_END(config, strategy, fordyca);
