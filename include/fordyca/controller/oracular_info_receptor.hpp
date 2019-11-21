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

#ifndef INCLUDE_FORDYCA_CONTROLLER_ORACULAR_INFO_RECEPTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_ORACULAR_INFO_RECEPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rcppsw { namespace ta {
class polled_task;
class bi_tdgraph_executive;
}} // namespace rcppsw::ta

NS_START(fordyca);
namespace support { namespace oracle {
class tasking_oracle;
class entities_oracle;
}} // namespace support::oracle
namespace ds {
class dpo_store;
} /* namespace ds */
NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class oracular_info_receptor
 * \ingroup fordyca controller
 *
 * \brief Plugin/hook for controllers to receive information from the loop
 * functions about the following:
 *
 * - Task exec/interface estimates
 * - Block/cache locations
 */

class oracular_info_receptor final : public rer::client<oracular_info_receptor> {
 public:
  oracular_info_receptor(support::oracle::tasking_oracle* tasking_oracle,
                         support::oracle::entities_oracle* entities_oracle)
      : ER_CLIENT_INIT("fordyca.controller.oracular_info_receptor"),
        m_tasking_oracle(tasking_oracle),
        m_entities_oracle(entities_oracle) {}

  oracular_info_receptor(const oracular_info_receptor& s) = delete;
  oracular_info_receptor& operator=(const oracular_info_receptor& s) = delete;

  /**
   * \brief Update the \ref dpo_store with the current set of oracular entities.
   *
   * Unlike the tasking oracle part of the plugin, which processes updates
   * whenever we change tasks (complete/abort), we need to synchronously
   * (per-timestep) update our perception of entities in the environment by
   * calling this function in the controller.
   */
  void dpo_store_update(ds::dpo_store* store);

  bool entities_blocks_enabled(void) const RCSW_PURE;
  bool entities_caches_enabled(void) const RCSW_PURE;
  bool tasking_enabled(void) const RCSW_PURE;

  void tasking_hooks_register(rta::bi_tdgraph_executive* executive);

 private:
  /**
   * \brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just aborted.
   */
  void task_abort_cb(rta::polled_task* task);

  /**
   * \brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just finished.
   */
  void task_finish_cb(rta::polled_task* task);

  void exec_est_update(rta::polled_task* task);
  void int_est_update(rta::polled_task* task);

  /* clang-format off */
  support::oracle::tasking_oracle*  m_tasking_oracle;
  support::oracle::entities_oracle* m_entities_oracle;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ORACULAR_INFO_RECEPTOR_HPP_ */
