/**
 * @file acquire_block_fsm.cpp
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
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth0/block_selector.hpp"
#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_block_fsm::acquire_block_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa,
    std::shared_ptr<representation::perceived_arena_map> map)
    : acquire_goal_fsm(server,
                       saa,
                       map,
                       std::bind(&acquire_block_fsm::block_detected_cb, this)),
      mc_nest_center(params->nest_center) {
  client::insmod("acquire_block_fsm",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
  goal_acquired_cb(std::bind(&acquire_block_fsm::block_acquired_cb,
                             this,
                             std::placeholders::_1));
}
/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_block_fsm::block_detected_cb(void) const {
  return saa_subsystem()->sensing()->block_detected();
} /* block_detected_cb() */

bool acquire_block_fsm::block_acquired_cb(bool explore_result) const {
  if (explore_result) {
    ER_ASSERT(saa_subsystem()->sensing()->block_detected(),
              "FATAL: No block detected after successful exploration?");
    return true;
  } else {
    if (saa_subsystem()->sensing()->block_detected()) {
      return true;
    }
    ER_WARN("WARNING: Robot arrived at goal, but no block was detected.");
    return false;
  }
} /* block_acquired_cb() */

bool acquire_block_fsm::acquire_known_goal(void) {
  std::list<representation::perceived_block> blocks = map()->perceived_blocks();
  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */

  if (blocks.empty() && !vector_fsm().task_running()) {
    return false;
  }

  if (!blocks.empty() && !vector_fsm().task_running()) {
    /*
     * If we get here, we must know of some blocks, but not be currently
     * vectoring toward any of them.
     */
    controller::depth0::block_selector selector(client::server_ref(),
                                                mc_nest_center);
    representation::perceived_block best =
        selector.calc_best(blocks, base_sensors()->position());
    ER_NOM("Vector towards best block: %d@(%zu, %zu)=%f",
           best.ent->id(),
           best.ent->discrete_loc().first,
           best.ent->discrete_loc().second,
           best.density.last_result());
    tasks::vector_argument v(vector_fsm::kBLOCK_ARRIVAL_TOL,
                             best.ent->real_loc());
    explore_fsm().task_reset();
    vector_fsm().task_reset();
    vector_fsm().task_start(&v);
  }

  /* we are vectoring */
  if (!vector_fsm().task_finished()) {
    vector_fsm().task_execute();
  }

  if (vector_fsm().task_finished()) {
    vector_fsm().task_reset();
    return block_acquired_cb(false);
  }
  return false;
} /* acquire_known_block() */

NS_END(controller, fordyca);
