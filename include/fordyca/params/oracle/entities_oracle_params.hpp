/**
 * @file entities_oracle_params.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/base_params.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, oracle);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct entities_oracle_params
 * @ingroup fordyca params oracle
 *
 * @brief Parameters for all-seeing oracle of entity location/size/etc.
 */
struct entities_oracle_params : public rparams::base_params {
  bool caches_enabled{false};
  bool blocks_enabled{false};
};

NS_END(oracle, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ORACLE_ENTITIES_ORACLE_PARAMS_HPP_ */
