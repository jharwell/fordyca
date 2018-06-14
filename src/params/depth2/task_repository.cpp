/**
 * @file task_repository.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/depth2/task_repository.hpp"
#include "fordyca/params/depth2/exec_estimates_parser.hpp"
#include "rcppsw/task_allocation/executive_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth2);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
task_repository::task_repository(
    const std::shared_ptr<rcppsw::er::server>& server)
    : xml_param_repository(server) {
  register_parser<exec_estimates_parser, exec_estimates_params>(
      exec_estimates_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<ta::executive_xml_parser, ta::executive_params>(
      ta::executive_xml_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
}

NS_END(depth2, params, fordyca);
