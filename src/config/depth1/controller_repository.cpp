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
#include "fordyca/config/depth1/controller_repository.hpp"
#include "fordyca/config/cache_sel/cache_sel_matrix_parser.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/ta/config/xml/task_alloc_parser.hpp"
#include "rcppsw/ta/config/xml/task_executive_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
controller_repository::controller_repository(void) {
  parser_register<cache_sel::cache_sel_matrix_parser,
                  cache_sel::cache_sel_matrix_config>(
      cache_sel::cache_sel_matrix_parser::kXMLRoot,
      cache_sel::cache_sel_matrix_parser::kHeader1);
  parser_register<rta::config::xml::task_alloc_parser,
                  rta::config::task_alloc_config>(
      rta::config::xml::task_alloc_parser::kXMLRoot,
      rconfig::xml::xml_config_parser::kHeader1);
  parser_register<rta::config::xml::task_executive_parser,
                  rta::config::task_executive_config>(
      rta::config::xml::task_executive_parser::kXMLRoot,
      rconfig::xml::xml_config_parser::kHeader1);

  parser_find<rta::config::xml::task_alloc_parser>(
      rta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("generalist");
  parser_find<rta::config::xml::task_alloc_parser>(
      rta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("collector");
  parser_find<rta::config::xml::task_alloc_parser>(
      rta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("harvester");
}

NS_END(depth1, config, fordyca);
