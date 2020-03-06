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
#include "fordyca/controller/oracular_info_receptor.hpp"

#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/polled_task.hpp"
#include "cosm/ta/time_estimate.hpp"

#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"
#include "cosm/oracle/entities_oracle.hpp"
#include "cosm/oracle/tasking_oracle.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct dpo_store_updater
 * \ingroup controller
 *
 * \brief Updates the \ref dpo_store with a single (possibly new) entity from
 * the list of entities we have been handed by the \ref entities_oracle.
 */
struct dpo_store_updater {
  explicit dpo_store_updater(ds::dpo_store* const s) : store(s) {}

  void operator()(crepr::base_block2D* block) const {
    events::block_found_visitor e(block);
    e.visit(*store);
  }
  void operator()(cfrepr::base_cache* cache) const {
    events::cache_found_visitor e(cache);
    e.visit(*store);
  }

  ds::dpo_store* const store;
};

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracular_info_receptor::dpo_store_update(ds::dpo_store* const store) {
  if (entities_blocks_enabled()) {
    /*
     * Segfault if this is in the loop rather than a dedicated variable due to
     * shared_ptr usage.
     */
    auto blocks = m_entities_oracle->ask("entities.blocks");
    if (!(*blocks).empty()) {
      ER_DEBUG(
          "Blocks in receptor: [%s]",
          support::oracle::entities_oracle::result_to_string(*blocks).c_str());
      ER_DEBUG("Blocks in DPO store: [%s]",
               rcppsw::to_string(store->blocks()).c_str());
    }
    for (auto& e : *blocks) {
      boost::apply_visitor(dpo_store_updater(store), e);
    } /* for(&e..) */
  }
  if (entities_caches_enabled()) {
    /*
     * Segfault if this is in the loop rather than a dedicated variable due to
     * shared_ptr usage.
     */
    auto caches = m_entities_oracle->ask("entities.caches");
    if (!(*caches).empty()) {
      ER_DEBUG(
          "Caches in receptor: [%s]",
          support::oracle::entities_oracle::result_to_string(*caches).c_str());
      ER_DEBUG("Caches in DPO store: [%s]",
               rcppsw::to_string(store->caches()).c_str());
    }
    for (auto& e : *caches) {
      boost::apply_visitor(dpo_store_updater(store), e);
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
  if (m_tasking_oracle->update_exec_ests()) {
    exec_est_update(task);
  }
  if (m_tasking_oracle->update_int_ests()) {
    int_est_update(task);
  }
} /* task_abort_cb() */

void oracular_info_receptor::task_finish_cb(cta::polled_task* const task) {
  if (m_tasking_oracle->update_exec_ests()) {
    exec_est_update(task);
  }
  if (m_tasking_oracle->update_int_ests()) {
    int_est_update(task);
  }
} /* task_finish_cb() */

void oracular_info_receptor::exec_est_update(cta::polled_task* const task) {
  auto exec_result = m_tasking_oracle->ask("exec_est." + task->name());
  ER_ASSERT(exec_result,
            "Bad oracle query 'exec_est.%s': no such task",
            task->name().c_str());
  auto oracle_exec_est = boost::get<cta::time_estimate>(exec_result.get());
  RCSW_UNUSED int exec_old = task->task_exec_estimate().v();
  task->exec_estimate_update(rtypes::timestep(oracle_exec_est.v()));
  ER_INFO("Update 'exec_est.%s' with oracular estimate %d: %d -> %d",
          task->name().c_str(),
          oracle_exec_est.v(),
          exec_old,
          task->task_exec_estimate().v());
} /* exec_est_update() */

void oracular_info_receptor::int_est_update(cta::polled_task* const task) {
  auto int_result = m_tasking_oracle->ask("interface_est." + task->name());
  ER_ASSERT(int_result,
            "Bad oracle query 'interface_est.%s': no such task",
            task->name().c_str());
  auto oracle_int_est = boost::get<cta::time_estimate>(int_result.get());
  RCSW_UNUSED int int_old = task->task_interface_estimate(0).v();
  task->interface_estimate_update(0, rtypes::timestep(oracle_int_est.v()));
  ER_INFO("Update 'interface_est.%s' with oracular estimate %d: %d -> %d",
          task->name().c_str(),
          oracle_int_est.v(),
          int_old,
          task->task_interface_estimate(0).v());
} /* int_est_update() */

bool oracular_info_receptor::entities_blocks_enabled(void) const {
  return m_entities_oracle->blocks_enabled();
} /* entities_blocks_enabled() */

bool oracular_info_receptor::entities_caches_enabled(void) const {
  return m_entities_oracle->caches_enabled();
} /* entities_caches_enabled() */

bool oracular_info_receptor::tasking_enabled(void) const {
  return nullptr != m_tasking_oracle;
} /* tasking_enabled() */

NS_END(controller, fordyca);
