/**
 * \file redist_governor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_REDIST_GOVERNOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_REDIST_GOVERNOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"
#include "fordyca/config/arena/block_redist_governor_config.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class redist_governor
 * \ingroup support block_dist
 *
 * \brief Supervises block distribution by determining the conditions under
 * which blocks should continue to be distributed, or if the # of blocks in the
 * arena should become finite. Conditions are one of:
 *
 * - Certain # of blocks have been collected (any type).
 * - The swarm has converged, according to some measure.
 * - A specified number of timesteps has elapsed.
 *
 * The distribution status switch policy can be specified to be allowed to
 * change multiple times, or to only occur at most once, like a switch flip (for
 * some trigger types).
 */
class redist_governor : public rer::client<redist_governor> {
 public:
  static constexpr char kStatusSwitchPolicySingle[] = "single";
  static constexpr char kStatusSwitchPolicyMulti[] = "multi";
  static constexpr char kTriggerNull[] = "";
  static constexpr char kTriggerTime[] = "timestep";
  static constexpr char kTriggerBlockCount[] = "block_count";
  static constexpr char kTriggerConvergence[] = "convergence";

  explicit redist_governor(
      const config::arena::block_redist_governor_config* config);

  /**
   * \brief Update the distribution status according to the policy parameters.
   *
   * \param t Current timestep.
   * \param blocks_collected # blocks collected so far.
   * \param convergence_status Current swarm convergence status.
   */
  void update(rtypes::timestep t,
              uint blocks_collected,
              bool convergence_status);
  bool dist_status(void) const { return m_dist_status; }

 private:
  /* clang-format off */
  const config::arena::block_redist_governor_config mc_config;

  bool                                              m_dist_status{true};
  /* clang-format on */
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_REDIST_GOVERNOR_HPP_ */
