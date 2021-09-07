/**
 * \file tv_manager_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CONFIG_TV_TV_MANAGER_CONFIG_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CONFIG_TV_TV_MANAGER_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/tv/config/population_dynamics_config.hpp"

#include "fordyca/support/config/tv/env_dynamics_config.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, config, tv);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct tv_manager_config
 * \ingroup support config tv
 *
 * \brief Configuration for the \ref tv_manager.
 */
struct tv_manager_config final : public rconfig::base_config {
  config::tv::env_dynamics_config env_dynamics{};
  ctv::config::population_dynamics_config population_dynamics{};
};

NS_END(tv, config, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CONFIG_TV_TV_MANAGER_CONFIG_HPP_ */
