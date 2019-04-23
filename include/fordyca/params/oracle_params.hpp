/**
 * @file oracle_params.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_ORACLE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ORACLE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct oracle_params
 * @ingroup fordyca params
 *
 * @brief Parameters for the various oracles that can be employed during
 * simulation.
 */
struct oracle_params : public rcppsw::params::base_params {
  bool enabled{false};
  /**
   * @brief Should the all-knowing oracle be used when updating task execution
   * time estimates? Only applicable to certain controllers.
   */
  bool task_exec_ests{false};

  /**
   * @brief Should the all-knowing oracle be used when updating task interface
   * time estimates? Only applicable to certain controllers.
   */
  bool task_interface_ests{false};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ORACLE_PARAMS_HPP_ */
