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
