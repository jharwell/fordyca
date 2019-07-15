/**
 * @file depth1/tasking_initializer.hpp
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
#include <memory>

#include "fordyca/nsalias.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace ta {
class polled_task;
class bi_tdgraph_executive;
class bi_tdgraph;
}}
NS_START(fordyca);
namespace config {
struct oracle_config;
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

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tasking_initializer
 * @ingroup fordyca controller depth1
 *
 * @brief A helper class to offload initialization of the task tree for depth1
 * foraging.
 */
class tasking_initializer : public rer::client<tasking_initializer> {
 public:
  tasking_initializer(const controller::block_sel_matrix* bsel_matrix,
                      const controller::cache_sel_matrix* csel_matrix,
                      controller::saa_subsystem* saa,
                      base_perception_subsystem* perception) RCSW_COLD;

  ~tasking_initializer(void) override RCSW_COLD;
  tasking_initializer& operator=(const tasking_initializer& other) = delete;
  tasking_initializer(const tasking_initializer& other) = delete;

  std::unique_ptr<rta::bi_tdgraph_executive>
  operator()(const config::depth1::controller_repository& config_repo) RCSW_COLD;

 protected:
  using tasking_map = std::map<std::string, rta::polled_task*>;

  RCSW_COLD const base_perception_subsystem* perception(void) const { return m_perception; }
  RCSW_COLD base_perception_subsystem* perception(void) { return m_perception; }

  RCSW_COLD controller::saa_subsystem* saa_subsystem(void) const { return m_saa; }
  RCSW_COLD const class block_sel_matrix* block_sel_matrix(void) const { return mc_bsel_matrix; }

  RCSW_COLD tasking_map depth1_tasks_create(
      const config::depth1::controller_repository& config_repo,
      rta::bi_tdgraph* graph);
  RCSW_COLD void depth1_exec_est_init(
      const config::depth1::controller_repository& config_repo,
      const tasking_map& map,
      rta::bi_tdgraph* graph);
  const class cache_sel_matrix* cache_sel_matrix(void) const { return mc_csel_matrix; }

 private:
  /* clang-format off */
  controller::saa_subsystem* const          m_saa;
  base_perception_subsystem* const          m_perception;
  const controller::cache_sel_matrix* const mc_csel_matrix;
  const controller::block_sel_matrix* const mc_bsel_matrix;
  /* clang-format on */
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_ */
