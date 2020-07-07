/**
 * \file depth1/task_executive_builder.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_EXECUTIVE_BUILDER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_EXECUTIVE_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <memory>

#include "fordyca/fordyca.hpp"
#include "rcppsw/er/client.hpp"
#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"
#include "rcppsw/math/rng.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta {
class polled_task;
class bi_tdgraph_executive;
namespace ds {
class bi_tdgraph;
} /* namespace ds */
}
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
class foraging_perception_subsystem;
NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_executive_builder
 * \ingroup controller depth1
 *
 * \brief A helper class to offload initialization of the task tree and
 * executive for depth1 foraging.
 */
class task_executive_builder : public rer::client<task_executive_builder> {
 public:
  task_executive_builder(const controller::block_sel_matrix* bsel_matrix,
                      const controller::cache_sel_matrix* csel_matrix,
                      crfootbot::footbot_saa_subsystem* saa,
                      foraging_perception_subsystem* perception) RCSW_COLD;

  ~task_executive_builder(void) override RCSW_COLD;
  task_executive_builder& operator=(const task_executive_builder&) = delete;
  task_executive_builder(const task_executive_builder&) = delete;

  RCSW_COLD std::unique_ptr<cta::bi_tdgraph_executive>
  operator()(const config::depth1::controller_repository& config_repo,
             rmath::rng* rng) RCSW_COLD;

 protected:
  using tasking_map = std::map<std::string, cta::polled_task*>;

  RCSW_COLD const foraging_perception_subsystem* perception(void) const { return m_perception; }
  RCSW_COLD foraging_perception_subsystem* perception(void) { return m_perception; }

  RCSW_COLD crfootbot::footbot_saa_subsystem* saa(void) const {
    return m_saa;
  }

  RCSW_COLD const class block_sel_matrix* block_sel_matrix(void) const {
    return mc_bsel_matrix;
  }

  RCSW_COLD const class cache_sel_matrix* cache_sel_matrix(void) const {
    return mc_csel_matrix;
  }

  RCSW_COLD tasking_map depth1_tasks_create(
      const config::depth1::controller_repository& config_repo,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

  RCSW_COLD void depth1_exec_est_init(
      const config::depth1::controller_repository& config_repo,
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

  RCSW_COLD void depth1_subtasks_init(
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

 private:
  /* clang-format off */
  const controller::cache_sel_matrix* const mc_csel_matrix;
  const controller::block_sel_matrix* const mc_bsel_matrix;

  crfootbot::footbot_saa_subsystem* const m_saa;
  foraging_perception_subsystem* const      m_perception;

  /* clang-format on */
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASK_EXECUTIVE_BUILDER_HPP_ */
