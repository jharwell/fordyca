/**
 * \file d1/task_executive_builder.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/subsystem/subsystem_fwd.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

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
namespace cosm::spatial {
class interference_tracker;
class nest_zone_tracker;
} /* namespace cosm::spatial */

namespace fordyca::controller::config::d1 {
 class controller_repository;
} /* namespace fordyca::controller::config::d1 */

NS_START(fordyca, controller, cognitive);
class cache_sel_matrix;
class block_sel_matrix;
class saa_subsystem;

NS_START(d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_executive_builder
 * \ingroup controller cognitive d1
 *
 * \brief A helper class to offload initialization of the task tree and
 * executive for d1 foraging.
 */
class task_executive_builder : public rer::client<task_executive_builder> {
 public:
  task_executive_builder(const controller::cognitive::block_sel_matrix* bsel_matrix,
                         const controller::cognitive::cache_sel_matrix* csel_matrix,
                         cspatial::interference_tracker* inta,
                         cspatial::nest_zone_tracker* nz,
                         csubsystem::saa_subsystemQ3D* saa,
                         fsperception::foraging_perception_subsystem* perception) RCPPSW_COLD;

  ~task_executive_builder(void) override RCPPSW_COLD;
  task_executive_builder& operator=(const task_executive_builder&) = delete;
  task_executive_builder(const task_executive_builder&) = delete;

  RCPPSW_COLD std::unique_ptr<cta::bi_tdgraph_executive>
  operator()(const config::d1::controller_repository& config_repo,
             rmath::rng* rng) RCPPSW_COLD;

 protected:
  using tasking_map = std::map<std::string, cta::polled_task*>;

  RCPPSW_COLD const fsperception::foraging_perception_subsystem* perception(void) const {
    return m_perception;
  }
  RCPPSW_COLD fsperception::foraging_perception_subsystem* perception(void) {
    return m_perception;
  }

  RCPPSW_COLD csubsystem::saa_subsystemQ3D* saa(void) const {
    return m_saa;
  }
  RCPPSW_COLD cspatial::interference_tracker* inta_tracker(void) const {
    return m_inta_tracker;
  }
  RCPPSW_COLD cspatial::nest_zone_tracker* nz_tracker(void) const {
    return m_nz_tracker;
  }


  RCPPSW_COLD const cognitive::block_sel_matrix* block_sel_matrix(void) const {
    return mc_bsel_matrix;
  }

  RCPPSW_COLD const cognitive::cache_sel_matrix* cache_sel_matrix(void) const {
    return mc_csel_matrix;
  }

  RCPPSW_COLD tasking_map d1_tasks_create(
      const config::d1::controller_repository& config_repo,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

  RCPPSW_COLD void d1_exec_est_init(
      const config::d1::controller_repository& config_repo,
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

  RCPPSW_COLD void d1_subtasks_init(
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);

 private:
  /* clang-format off */
  const controller::cognitive::cache_sel_matrix* const mc_csel_matrix;
  const controller::cognitive::block_sel_matrix* const mc_bsel_matrix;

  cspatial::interference_tracker* const                m_inta_tracker;
  cspatial::nest_zone_tracker* const                   m_nz_tracker;
  csubsystem::saa_subsystemQ3D* const                  m_saa;
  fsperception::foraging_perception_subsystem* const   m_perception;
  /* clang-format on */
};

NS_END(cognitive, d1, controller, fordyca);
