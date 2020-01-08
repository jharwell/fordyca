/**
 * \file tasking_oracle.cpp
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
#include "fordyca/support/oracle/tasking_oracle.hpp"

#include <functional>

#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/ta/polled_task.hpp"

#include "fordyca/config/oracle/tasking_oracle_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, oracle);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
tasking_oracle::tasking_oracle(
    const config::oracle::tasking_oracle_config* const config,
    const cta::ds::bi_tdgraph* const graph)
    : ER_CLIENT_INIT("fordyca.support.tasking_oracle"),
      mc_exec_ests(config->task_exec_ests),
      mc_int_ests(config->task_interface_ests) {
  auto cb = [&](const cta::polled_task* task) {
    m_map.insert({"exec_est." + task->name(), task->task_exec_estimate()});
    m_map.insert(
        {"interface_est." + task->name(), task->task_interface_estimate(0)});
    ER_WARN("Assuming all tasks have at most 1 interface");
  };
  graph->walk(cb);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<tasking_oracle::variant_type> tasking_oracle::ask(
    const std::string& query) const {
  auto it = m_map.find(query);
  return (it != m_map.end()) ? boost::make_optional(it->second)
                             : boost::optional<variant_type>();
} /* ask() */

void tasking_oracle::listener_add(cta::bi_tdgraph_executive* const executive) {
  executive->task_abort_notify(
      std::bind(&tasking_oracle::task_abort_cb, this, std::placeholders::_1));
  executive->task_finish_notify(
      std::bind(&tasking_oracle::task_finish_cb, this, std::placeholders::_1));
} /* listener_add() */

void tasking_oracle::task_finish_cb(const cta::polled_task* task) {
  auto& exec_est = boost::get<cta::time_estimate>(
      m_map.find("exec_est." + task->name())->second);
  RCSW_UNUSED int exec_old = exec_est.v();
  exec_est.calc(task->task_exec_estimate());

  ER_DEBUG("Update exec_est.%s on finish: %d -> %d",
           task->name().c_str(),
           exec_old,
           exec_est.v());

  auto& int_est = boost::get<cta::time_estimate>(
      m_map.find("interface_est." + task->name())->second);
  RCSW_UNUSED int int_old = int_est.v();

  /* Assuming 1 interface! */
  int_est.calc(task->task_interface_estimate(0));

  ER_DEBUG("Update interface_est.%s on finish: %d -> %d",
           task->name().c_str(),
           int_old,
           int_est.v());
} /* task_finish_cb() */

void tasking_oracle::task_abort_cb(const cta::polled_task* task) {
  /*
   * \todo Updating task exec/interface estimates on abort is a little dicey, as
   * it can cause tasks that just failed to be re-attempted because they have a
   * much lower estimate than successful tasks. This is the current non-oracle
   * behavior, so we duplicate it here; it should be less of an issue, as we
   * should have a lot of updates coming in.
   *
   * Whether updating estimates on abort actually matters is tracked by #416,
   * and will be eventually be implemented.
   */
  auto& exec_est = boost::get<cta::time_estimate>(
      m_map.find("exec_est." + task->name())->second);
  RCSW_UNUSED int exec_old = exec_est.v();
  exec_est.calc(task->task_exec_estimate());

  ER_DEBUG("Update exec_est.%s on abort: %d -> %d",
           task->name().c_str(),
           exec_old,
           exec_est.v());

  auto& int_est = boost::get<cta::time_estimate>(
      m_map.find("interface_est." + task->name())->second);
  RCSW_UNUSED int int_old = int_est.v();

  /* Assuming 1 interface! */
  int_est.calc(task->task_interface_estimate(0));

  ER_DEBUG("Update interface_est.%s on abort: %d -> %d",
           task->name().c_str(),
           int_old,
           int_est.v());
} /* task_abort_cb() */

NS_END(oracle, support, fordyca);
