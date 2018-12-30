/**
 * @file tasking_initializer.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class polled_task;
class bi_tdgraph_executive;
class bi_tdgraph;
}}
NS_START(fordyca);
namespace params {
struct oracle_params;
namespace depth1 { class controller_repository; }
}

namespace ds {
class dpo_store;
}

NS_START(controller);
class cache_sel_matrix;
class block_sel_matrix;
class saa_subsystem;
class base_perception_subsystem;
NS_START(depth1);

namespace ta = rcppsw::task_allocation;
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tasking_initializer
 * @ingroup controller depth1
 *
 * @brief A helper class to offload initialization of the task tree for depth1
 * foraging.
 */
class tasking_initializer : public er::client<tasking_initializer> {
 public:
  tasking_initializer(const controller::block_sel_matrix* bsel_matrix,
                      const controller::cache_sel_matrix* csel_matrix,
                      controller::saa_subsystem* saa,
                      base_perception_subsystem* perception);

  ~tasking_initializer(void) override;
  tasking_initializer& operator=(const tasking_initializer& other) = delete;
  tasking_initializer(const tasking_initializer& other) = delete;

  std::unique_ptr<ta::bi_tdgraph_executive>
  operator()(const params::depth1::controller_repository& param_repo);

  using tasking_map = std::map<std::string, ta::polled_task*>;

 protected:
  const base_perception_subsystem* perception(void) const { return m_perception; }
  base_perception_subsystem* perception(void) { return m_perception; }

  const ds::dpo_store* dpo_store(void) const;
  ds::dpo_store* dpo_store(void);

  controller::saa_subsystem* saa_subsystem(void) const { return m_saa; }
  ta::bi_tdgraph* graph(void) { return m_graph; }
  const ta::bi_tdgraph* graph(void) const { return m_graph; }
  const class block_sel_matrix* block_sel_matrix(void) const { return mc_bsel_matrix; }

  tasking_map depth1_tasks_create(
      const params::depth1::controller_repository& param_repo);
  void depth1_exec_est_init(
      const params::depth1::controller_repository& param_repo,
      const tasking_map& map);
  const class cache_sel_matrix* cache_sel_matrix(void) const { return mc_csel_matrix; }

 private:
  // clang-format off
  controller::saa_subsystem* const                m_saa;
  base_perception_subsystem* const                m_perception;
  const controller::cache_sel_matrix* const mc_csel_matrix;
  const controller::block_sel_matrix* const mc_bsel_matrix;

  ta::bi_tdgraph*                                 m_graph;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_ */
