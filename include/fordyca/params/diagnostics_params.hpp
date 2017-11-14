/**
 * @file diagnostics_params.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_PARAMS_DIAGNOSTICS_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_DIAGNOSTICS_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/common/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct diagnostics_params : public rcppsw::common::base_params {
  diagnostics_params(void) : block_fname(), random_fname(), distance_fname(),
                             depth0_fname(), depth1_fname(), n_robots() {}

  std::string block_fname;
  std::string random_fname;
  std::string distance_fname;
  std::string depth0_fname;
  std::string depth1_fname;
  size_t n_robots;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DIAGNOSTICS_PARAMS_HPP_ */
