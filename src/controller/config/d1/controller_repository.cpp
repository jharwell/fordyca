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
#include "fordyca/controller/config/d1/controller_repository.hpp"

#include "cosm/hal/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"
#include "cosm/ta/config/xml/task_alloc_parser.hpp"

#include "fordyca/controller/config/cache_sel/cache_sel_matrix_parser.hpp"
#include "fordyca/tasks/d0/foraging_task.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d1);
namespace rtconfig = cta::config;
namespace rtcxml = cta::config::xml;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
controller_repository::controller_repository(void) {
  parser_register<cache_sel::cache_sel_matrix_parser,
                  cache_sel::cache_sel_matrix_config>(
      cache_sel::cache_sel_matrix_parser::kXMLRoot);

  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd0::foraging_task::kGeneralistName);
  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd1::foraging_task::kCollectorName);
  parser_find<rtcxml::task_alloc_parser>(rtcxml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd1::foraging_task::kHarvesterName);

  parser_find<chsubsystem::config::xml::sensing_subsystemQ3D_parser>(
      chsubsystem::config::xml::sensing_subsystemQ3D_parser::kXMLRoot)
      ->env_detection_add("cache");
}

NS_END(d1, config, controller, fordyca);
