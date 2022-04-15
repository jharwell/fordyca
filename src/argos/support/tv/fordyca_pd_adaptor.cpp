/**
 * \file fordyca_pd_adaptor.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "fordyca/argos/support/tv/fordyca_pd_adaptor.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/sim_block3D.hpp"

#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
fordyca_pd_adaptor::fordyca_pd_adaptor(
    const ctv::config::population_dynamics_config* config,
    cpargos::swarm_manager_adaptor* sm,
    env_dynamics_type* envd,
    carena::caching_arena_map* map,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.argos support.tv.fordyca_pd_adaptor"),
      pd_adaptor<cpcontroller::controller2D>(config,
                                             sm,
                                             envd,
                                             rmath::vector2d(map->xrsize(),
                                                             map->yrsize()),
                                             rng),
      m_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fordyca_pd_adaptor::pre_kill_cleanup(cpcontroller::controller2D* controller) {
  auto* foraging = static_cast<controller::foraging_controller*>(controller);
  /*
   * If the robot is carrying a block, drop/distribute it in the arena to avoid
   * it getting permanently lost when the it is removed.
   */
  if (foraging->is_carrying_block()) {
    ER_INFO("Kill victim robot %s is carrying block%d",
            foraging->GetId().c_str(),
            foraging->block()->id().v());
    auto it = std::find_if(
        m_map->blocks().begin(), m_map->blocks().end(), [&](const auto& b) {
          return foraging->block()->id() == b->id();
        });
    /*
     * We are not REALLY holding all the arena map locks, but since population
     * dynamics are always applied AFTER all robots have had their control steps
     * run, we are in a non-concurrent context, so no reason to grab them.
     */
    caops::free_block_drop_visitor adrop_op(
        *it,
        rmath::dvec2zvec(foraging->rpos2D(), m_map->grid_resolution().v()),
        m_map->grid_resolution(),
        carena::locking::ekALL_HELD);

    adrop_op.visit(*m_map);
  }
} /* pre_kill_cleanup() */

NS_END(tv, support, argos, fordyca);
