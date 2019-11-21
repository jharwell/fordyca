/**
 * \file controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "rcppsw/ta/config/xml/task_alloc_parser.hpp"
#include "rcppsw/ta/config/xml/task_executive_parser.hpp"

#include "fordyca/config/cache_sel/cache_sel_matrix_parser.hpp"
#include "fordyca/fordyca.hpp"

#include "cosm/subsystem/config/xml/sensing_subsystem2D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, depth1);
namespace csconfig = csubsystem::config;
namespace cscxml = csconfig::xml;
namespace rtconfig = rta::config;
namespace rtcxml = rta::config::xml;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
controller_repository::controller_repository(void) {
  parser_register<cache_sel::cache_sel_matrix_parser,
                  cache_sel::cache_sel_matrix_config>(
      cache_sel::cache_sel_matrix_parser::kXMLRoot);
  parser_register<rtcxml::task_alloc_parser, rtconfig::task_alloc_config>(
      rtcxml::task_alloc_parser::kXMLRoot);
  parser_register<rtcxml::task_executive_parser, rtconfig::task_executive_config>(
      rtcxml::task_executive_parser::kXMLRoot);

  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("generalist");
  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("collector");
  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add("harvester");

  parser_find<cscxml::sensing_subsystem2D_parser>(
      cscxml::sensing_subsystem2D_parser::kXMLRoot)
      ->ground_detection_add("cache");
}

NS_END(depth1, config, fordyca);
