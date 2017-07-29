/**
 * @file actuator_param_parser.hpp
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

#ifndef INCLUDE_ACTUATOR_PARAM_PARSER_HPP_
#define INCLUDE_ACTUATOR_PARAM_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
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
class actuator_param_parser: public base_param_parser {
 public:
  actuator_param_parser(void) {}

  const struct actuator_params& parse(argos::TConfigurationNode& node);

 private:
  struct actuator_params m_params;
};

NS_END(fordyca);

#endif /* INCLUDE_ACTUATOR_PARAM_PARSER_HPP_ */
