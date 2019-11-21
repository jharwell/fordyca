/**
 * \file oracle_manager_config.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/config/oracle/tasking_oracle_config.hpp"
#include "fordyca/config/oracle/entities_oracle_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, oracle);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct oracle_manager_config
 * \ingroup fordyca config oracle
 *
 * \brief Parameters for the various oracles that can be employed during
 * simulation.
 */
struct oracle_manager_config final : public rconfig::base_config {
  struct tasking_oracle_config tasking{};
  struct entities_oracle_config entities{};
};

NS_END(oracle, config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_ORACLE_ORACLE_MANAGER_CONFIG_HPP_ */
