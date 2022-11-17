/**
 * \file controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
