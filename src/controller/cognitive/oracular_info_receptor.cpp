/**
 * \file oracular_info_receptor.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/oracular_info_receptor.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/tasking_oracle.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/polled_task.hpp"
#include "cosm/ta/time_estimate.hpp"

#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller, cognitive);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracular_info_receptor::dpo_store_update(ds::dpo_store* const store) {
  if (entities_blocks_enabled()) {
    auto blocks = mc_oracle->blocks()->ask();
    if (!blocks.empty()) {
      ER_DEBUG("Blocks in receptor: [%s]",
               cforacle::foraging_oracle::blocks_oracle_type::knowledge_to_string(
                   "b", blocks)
                   .c_str());
      ER_DEBUG("Blocks in DPO store: [%s]",
               rcppsw::to_string(store->blocks()).c_str());
    }
    for (auto* b : blocks) {
      events::block_found_visitor visitor(b);
      visitor.visit(*store);
    } /* for(&e..) */
  }
  if (entities_caches_enabled()) {
    auto caches = mc_oracle->caches()->ask();
    if (!caches.empty()) {
      ER_DEBUG("Caches in receptor: [%s]",
               cforacle::foraging_oracle::caches_oracle_type::knowledge_to_string(
                   "c", caches)
                   .c_str());
      ER_DEBUG("Caches in DPO store: [%s]",
               rcppsw::to_string(store->caches()).c_str());
    }
    for (auto* c : caches) {
      events::cache_found_visitor visitor(c);
      visitor.visit(*store);
    } /* for(&e..) */
  }
} /* dpo_store_update() */

void oracular_info_receptor::tasking_hooks_register(
    cta::bi_tdgraph_executive* const executive) {
  executive->task_abort_notify(std::bind(
      &oracular_info_receptor::task_abort_cb, this, std::placeholders::_1));
  executive->task_finish_notify(std::bind(
      &oracular_info_receptor::task_finish_cb, this, std::placeholders::_1));
} /* tasking_hooks_register() */

void oracular_info_receptor::task_abort_cb(cta::polled_task* const task) {
  if (mc_oracle->tasking()->update_exec_ests()) {
    exec_est_update(task);
  }
  if (mc_oracle->tasking()->update_int_ests()) {
    int_est_update(task);
  }
} /* task_abort_cb() */

void oracular_info_receptor::task_finish_cb(cta::polled_task* const task) {
  if (mc_oracle->tasking()->update_exec_ests()) {
    exec_est_update(task);
  }
  if (mc_oracle->tasking()->update_int_ests()) {
    int_est_update(task);
  }
} /* task_finish_cb() */

void oracular_info_receptor::exec_est_update(cta::polled_task* const task) {
  auto exec_result = mc_oracle->tasking()->ask("exec_est." + task->name());
  ER_ASSERT(exec_result,
            "Bad oracle query 'exec_est.%s': no such task",
            task->name().c_str());
  auto oracle_exec_est = boost::get<cta::time_estimate>(exec_result.get());
  RCPPSW_UNUSED int exec_old = task->task_exec_estimate().v();
  task->exec_estimate_update(rtypes::timestep(oracle_exec_est.v()));
  ER_INFO("Update 'exec_est.%s' with oracular estimate %d: %d -> %d",
          task->name().c_str(),
          oracle_exec_est.v(),
          exec_old,
          task->task_exec_estimate().v());
} /* exec_est_update() */

void oracular_info_receptor::int_est_update(cta::polled_task* const task) {
  auto int_result = mc_oracle->tasking()->ask("interface_est." + task->name());
  ER_ASSERT(int_result,
            "Bad oracle query 'interface_est.%s': no such task",
            task->name().c_str());
  auto oracle_int_est = boost::get<cta::time_estimate>(int_result.get());
  RCPPSW_UNUSED int int_old = task->task_interface_estimate(0).v();
  task->interface_estimate_update(0, rtypes::timestep(oracle_int_est.v()));
  ER_INFO("Update 'interface_est.%s' with oracular estimate %d: %d -> %d",
          task->name().c_str(),
          oracle_int_est.v(),
          int_old,
          task->task_interface_estimate(0).v());
} /* int_est_update() */

bool oracular_info_receptor::entities_blocks_enabled(void) const {
  auto blocks_it = mc_oracle->config()->entities.types.find("blocks");
  return mc_oracle->config()->entities.types.end() != blocks_it &&
         blocks_it->second;
} /* entities_blocks_enabled() */

bool oracular_info_receptor::entities_caches_enabled(void) const {
  auto caches_it = mc_oracle->config()->entities.types.find("caches");
  return mc_oracle->config()->entities.types.end() != caches_it &&
         caches_it->second;
} /* entities_caches_enabled() */

bool oracular_info_receptor::tasking_enabled(void) const {
  return nullptr != mc_oracle->tasking();
} /* tasking_enabled() */

NS_END(cognitive, controller, fordyca);
