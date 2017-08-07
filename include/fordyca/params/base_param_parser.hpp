/**
 * @file base_param_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_BASE_PARAM_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_BASE_PARAM_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <fstream>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/params/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_param_parser {
 public:
  base_param_parser(void) {}
  virtual ~base_param_parser(void) {}

  virtual void parse(__unused argos::TConfigurationNode& node) = 0;
  virtual void show(std::ostream& stream) = 0;
  virtual const struct base_params* get_results(void) { return NULL; }
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_BASE_PARAM_PARSER_HPP_ */
