/**
 * @file loop_function_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
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

#ifndef INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTION_REPOSITORY_HPP_
#define INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTION_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/xml_param_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class loop_function_repository : public rcppsw::params::xml_param_repository {
 public:
  loop_function_repository(void);
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTION_REPOSITORY_HPP_ */
