/**
 * @file parameter_parser.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <map>
#include "rcppsw/common/common.hpp"
#include "fordyca/fordyca_params.hpp"
#include "fordyca/base_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class parameter_parser {
 public:
  parameter_parser(void) : m_parsers() {}

  status_t add_category(const std::string& category, base_param_parser* parser);
  status_t parse_all(argos::TConfigurationNode& node);
  const struct base_params* get_params(const std::string& name) { return m_parsers[name]->get_results(); }

 private:
  std::map<std::string, base_param_parser*> m_parsers;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_ */
