/**
 * @file controller_repository.cpp
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
#include "fordyca/params/depth1/controller_repository.hpp"
#include "rcppsw/task_allocation/task_allocation_xml_parser.hpp"
#include "fordyca/params/cache_sel_matrix_parser.hpp"
#include "rcppsw/task_allocation/task_allocation_xml_parser.hpp"
#include "rcppsw/task_allocation/task_executive_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
controller_repository::controller_repository(void) {
  register_parser<cache_sel_matrix_parser,
                  cache_sel_matrix_params>(cache_sel_matrix_parser::kXMLRoot,
                                           cache_sel_matrix_parser::kHeader1);
    register_parser<ta::task_allocation_xml_parser, ta::task_allocation_params>(
      ta::task_allocation_xml_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<ta::task_executive_xml_parser, ta::task_executive_params>(
      ta::task_executive_xml_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);

  get_parser<ta::task_allocation_xml_parser>(
      ta::task_allocation_xml_parser::kXMLRoot)->exec_est_task_add("generalist");
  get_parser<ta::task_allocation_xml_parser>(
      ta::task_allocation_xml_parser::kXMLRoot)->exec_est_task_add("collector");
  get_parser<ta::task_allocation_xml_parser>(
      ta::task_allocation_xml_parser::kXMLRoot)->exec_est_task_add("harvester");
}

NS_END(depth1, params, fordyca);
