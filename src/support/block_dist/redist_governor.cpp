/**
 * @file redist_governor.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/support/block_dist/redist_governor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, block_dist);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
constexpr char redist_governor::kTriggerNull[];
constexpr char redist_governor::kTriggerTime[];
constexpr char redist_governor::kTriggerBlockCount[];
constexpr char redist_governor::kTriggerConvergence[];
constexpr char redist_governor::kStatusSwitchPolicySingle[];
constexpr char redist_governor::kStatusSwitchPolicyMulti[];

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
redist_governor::redist_governor(
    const params::arena::block_redist_governor_params* const params)
    : ER_CLIENT_INIT("fordyca.support.block_dist.redist_governor"),
      mc_params(*params) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void redist_governor::update(uint timestep,
                             uint blocks_collected,
                             bool convergence_status) {
  /* # blocks is always infinite */
  if (kTriggerNull == mc_params.trigger) {
    return;
  }
  /*
   * Can only be tripped once, so if already tripped avoid printing
   * diagnostic multiple times.
   */
  if (kTriggerTime == mc_params.trigger && timestep >= mc_params.timestep &&
      m_dist_status) {
    ER_INFO(
        "Redistribution disabled by trigger '%s': "
        "t=%u,n_blocks=%u,convergence=%d",
        kTriggerTime,
        timestep,
        blocks_collected,
        convergence_status);
    m_dist_status = false;
    return;
  }
  if (kTriggerBlockCount == mc_params.trigger &&
      blocks_collected >= mc_params.block_count && m_dist_status) {
    ER_INFO("Redistribution disabled by '%s': t=%u,n_blocks=%u,convergence=%d",
            kTriggerBlockCount,
            timestep,
            blocks_collected,
            convergence_status);
    m_dist_status = false;
    return;
  }
  if (kTriggerConvergence == mc_params.trigger) {
    if (kStatusSwitchPolicySingle == mc_params.recurrence_policy &&
        !m_dist_status) {
      return;
    }
    /*
     * For multi switch, we only redistribute blocks when the swarm is not
     * converged.
     */
    if (m_dist_status == !convergence_status) {
      return;
    }
    m_dist_status = !convergence_status;
    ER_INFO(
        "Redistribution=%d triggered by '%s': t=%u,n_blocks=%u,convergence=%d",
        m_dist_status,
        kTriggerConvergence,
        timestep,
        blocks_collected,
        convergence_status);
    return;
  }
  ER_FATAL_SENTINEL("Bad trigger type '%s'", mc_params.trigger.c_str());
} /* update() */

NS_END(block_dist, support, fordyca);
