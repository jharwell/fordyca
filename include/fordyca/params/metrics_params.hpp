/**
 * @file metrics_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_

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
struct metrics_params : public rcppsw::common::base_params {
  metrics_params(void) : root_dir(), output_dir(), block_fname(),
                         stateless_fname(), stateful_fname(),
                         distance_fname(), depth1_fname(), task_fname(),
                         n_robots() {}

  std::string root_dir;
  std::string output_dir;
  std::string block_fname;
  std::string stateless_fname;
  std::string stateful_fname;
  std::string distance_fname;
  std::string depth1_fname;
  std::string task_fname;
  size_t n_robots;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_ */
