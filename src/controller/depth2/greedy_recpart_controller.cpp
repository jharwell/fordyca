/**
 * @file greedy_recpartpart__controller.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"

#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
greedy_recpart_controller::greedy_recpart_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth2.greedy_recpart") {}

greedy_recpart_controller::~greedy_recpart_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void greedy_recpart_controller::ControlStep(void) {
  ndc_pusht();
  perception()->update(depth1::greedy_partitioning_controller::los());

  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(
      saa_subsystem()->sensing()->tick());

  executive()->run();
  ndc_pop();
} /* ControlStep() */

void greedy_recpart_controller::Init(ticpp::Element& node) {
  base_controller::Init(node);
  ndc_push();
  ER_INFO("Initializing");

  params::depth2::controller_repository param_repo;
  depth1::greedy_partitioning_controller::non_unique_init(node, &param_repo);

  executive(tasking_initializer(block_sel_matrix(),
                                cache_sel_matrix(),
                                saa_subsystem(),
                                perception())(&param_repo));

  executive()->task_alloc_notify(std::bind(&greedy_recpart_controller::task_alloc_cb,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

__rcsw_pure tasks::base_foraging_task* greedy_recpart_controller::current_task(
    void) {
  return dynamic_cast<tasks::base_foraging_task*>(executive()->current_task());
} /* current_task() */

__rcsw_pure const tasks::base_foraging_task* greedy_recpart_controller::current_task(
    void) const {
  return const_cast<greedy_recpart_controller*>(this)->current_task();
} /* current_task() */

void greedy_recpart_controller::task_alloc_cb(
    const ta::polled_task* const,
    const ta::bi_tab* const) {
  if (!m_bsel_exception_added) {
    block_sel_matrix()->sel_exceptions_clear();
  }
  m_bsel_exception_added = false;
} /* task_alloc_cb() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(greedy_recpart_controller,
                    "greedy_recpart_controller"); // NOLINT
#pragma clang diagnostic pop

NS_END(depth2, controller, fordyca);
