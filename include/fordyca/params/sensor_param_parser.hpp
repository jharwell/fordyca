/**
 * @file sensor_param_parser.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_PARAMS_SENSOR_PARAM_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMS_SENSOR_PARAM_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/params/base_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class sensor_param_parser: public base_param_parser {
 public:
  sensor_param_parser(void) : m_params() {}

  void parse(argos::TConfigurationNode& node);
  const struct sensor_params* get_results(void) { return m_params.get(); }
  void show(std::ostream& stream);

 private:
  std::unique_ptr<struct sensor_params> m_params;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_SENSOR_PARAM_PARSER_HPP_ */
