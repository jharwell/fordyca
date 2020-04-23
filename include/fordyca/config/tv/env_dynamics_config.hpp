/**
 * \file env_dynamics_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/config/base_env_dynamics_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, tv);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_dynamics_config
 * \ingroup config tv
 *
 * \brief Configuration for the \ref env_dynamics.
 */
struct env_dynamics_config final : public ctv::config::base_env_dynamics_config {
  rct::config::waveform_config cache_usage_penalty{};
};

NS_END(tv, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_TV_ENV_DYNAMICS_CONFIG_HPP_ */
