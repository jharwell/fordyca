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
#include "fordyca/controller/config/d2/controller_repository.hpp"

#include "cosm/ta/config/xml/task_alloc_parser.hpp"

#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
controller_repository::controller_repository(void) {
  parser_find<cta::config::xml::task_alloc_parser>(
      cta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd2::foraging_task::kCacheStarterName);
  parser_find<cta::config::xml::task_alloc_parser>(
      cta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd2::foraging_task::kCacheFinisherName);
  parser_find<cta::config::xml::task_alloc_parser>(
      cta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd2::foraging_task::kCacheTransfererName);
  parser_find<cta::config::xml::task_alloc_parser>(
      cta::config::xml::task_alloc_parser::kXMLRoot)
      ->exec_est_task_add(ftd2::foraging_task::kCacheCollectorName);
}

NS_END(d2, config, controller, fordyca);
