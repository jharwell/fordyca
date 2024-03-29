/**
 * \file d2/task_executive_builder.hpp
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
#include <string>
#include <memory>

#include "fordyca/controller/cognitive/d1/task_executive_builder.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::controller::config::d2 { class controller_repository; }

NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_executive_builder
 * \ingroup controller cognitive d2
 *
 * \brief A helper class to offload initialization of the task tree and
 * executive for d2 foraging.
 */
class task_executive_builder : public d1::task_executive_builder,
                               public rer::client<task_executive_builder> {
 public:
  task_executive_builder(
      const controller::cognitive::block_sel_matrix* bsel_matrix,
      const controller::cognitive::cache_sel_matrix* csel_matrix,
      cspatial::interference_tracker* inta,
      cspatial::nest_zone_tracker* nz,
      csubsystem::saa_subsystemQ3D* saa,
      fsperception::foraging_perception_subsystem* perception) RCPPSW_COLD;

  ~task_executive_builder(void) override RCPPSW_COLD;

  RCPPSW_COLD std::unique_ptr<cta::bi_tdgraph_executive>
  operator()(const config::d2::controller_repository& config_repo,
             rmath::rng* rng) RCPPSW_COLD;

  using d1::task_executive_builder::tasking_map;

 protected:
  RCPPSW_COLD tasking_map d2_tasks_create(
      const config::d2::controller_repository& config_repo,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng) RCPPSW_COLD;

  RCPPSW_COLD void d2_exec_est_init(
      const config::d2::controller_repository& config_repo,
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng) RCPPSW_COLD;

  RCPPSW_COLD void d2_subtasks_init(
      const tasking_map& map,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng);
};

NS_END(cognitive, d2, controller, fordyca);

