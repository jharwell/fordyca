/**
 * \file oracular_info_receptor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {
class polled_task;
class bi_tdgraph_executive;
} // namespace cosm::ta

namespace cosm::foraging::oracle {
class foraging_oracle;
} // namespace cosm::foraging::oracle


NS_START(fordyca, subsystem, perception);

namespace ds {
class dpo_store;
} /* namespace ds */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class oracular_info_receptor
 * \ingroup subsystem perception
 *
 * \brief Plugin/hook for controllers to receive information from the loop
 * functions about the following:
 *
 * - Task exec/interface estimates
 * - Block/cache locations
 */

class oracular_info_receptor final : public rer::client<oracular_info_receptor> {
 public:
  explicit oracular_info_receptor(const cforacle::foraging_oracle* oracle)
      : ER_CLIENT_INIT("fordyca.subsystem.perception.oracular_info_receptor"),
        mc_oracle(oracle) {}

  oracular_info_receptor(const oracular_info_receptor&) = delete;
  oracular_info_receptor& operator=(const oracular_info_receptor&) = delete;

  /**
   * \brief Update the \ref dpo_store with the current set of oracular entities.
   *
   * Unlike the tasking oracle part of the plugin, which processes updates
   * whenever we change tasks (complete/abort), we need to synchronously
   * (per-timestep) update our perception of entities in the environment by
   * calling this function in the controller.
   */
  void dpo_store_update(ds::dpo_store* store);

  bool entities_blocks_enabled(void) const RCPPSW_PURE;
  bool entities_caches_enabled(void) const RCPPSW_PURE;
  bool tasking_enabled(void) const RCPPSW_PURE;

  void tasking_hooks_register(cta::bi_tdgraph_executive* executive);

 private:
  /**
   * \brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just aborted.
   */
  void task_abort_cb(cta::polled_task* task);

  /**
   * \brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just finished.
   */
  void task_finish_cb(cta::polled_task* task);

  void exec_est_update(cta::polled_task* task);
  void int_est_update(cta::polled_task* task);

  /* clang-format off */
  const cforacle::foraging_oracle* mc_oracle;
  /* clang-format on */
};

NS_END(perception, subsystem,fordyca);

