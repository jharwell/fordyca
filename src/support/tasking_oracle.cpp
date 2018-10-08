/**
 * @file tasking_oracle.cpp
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
#include "fordyca/support/tasking_oracle.hpp"
#include <functional>

#include "rcppsw/task_allocation/bifurcating_tdgraph.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
tasking_oracle::tasking_oracle(const ta::bifurcating_tdgraph* const graph)
    : ER_CLIENT_INIT("fordyca.support.tasking_oracle") {
  graph->walk([&](const ta::polled_task* task) {
    m_map.insert({"exec_est." + task->name(), task->task_exec_estimate()});
  });
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
tasking_oracle::mapped_type tasking_oracle::ask(const std::string& query) const {
  return m_map.find(query)->second;
} /* ask() */

void tasking_oracle::listener_add(
    ta::bifurcating_tdgraph_executive* const executive) {
  executive->task_abort_notify(
      std::bind(&tasking_oracle::task_abort_cb, this, std::placeholders::_1));
  executive->task_finish_notify(
      std::bind(&tasking_oracle::task_finish_cb, this, std::placeholders::_1));
} /* listener_add() */

void tasking_oracle::task_finish_cb(const ta::polled_task* task) {
  auto& est = boost::get<ta::time_estimate>(
      m_map.find("exec_est." + task->name())->second);
  double old = est.last_result();
  est.calc(task->task_exec_estimate());

  ER_DEBUG("Update exec_est.%s on finish: %f -> %f",
           task->name().c_str(),
           old,
           est.last_result());
} /* task_finish_cb() */

void tasking_oracle::task_abort_cb(const ta::polled_task* task) {
  auto& est = boost::get<ta::time_estimate>(
      m_map.find("exec_est." + task->name())->second);
  double old = est.last_result();
  est.calc(task->task_exec_estimate());

  ER_DEBUG("Update exec_est.%s on abort: %f -> %f",
           task->name().c_str(),
           old,
           est.last_result());
} /* task_abort_cb() */

NS_END(support, fordyca);
