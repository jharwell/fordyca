/**
 * \file block_acquisition_validator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/block_acq_validator.hpp"

#include <numeric>

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
using bselm = controller::cognitive::block_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_acq_validator::block_acq_validator(
    const fspds::dp_block_map* map,
    const controller::cognitive::block_sel_matrix* matrix)
    : ER_CLIENT_INIT("fordyca.fsm.block_acq_validator"),
      mc_map(map),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool block_acq_validator::operator()(const rmath::vector2d& loc,
                                     const rtypes::type_uuid& id) const {
  const auto* block = mc_map->find(id);

  /* Sanity checks for acqusition */
  if (nullptr == block) {
    ER_WARN("Acquisition of free block%d@%s invalid: block unknown",
            id.v(),
            loc.to_str().c_str());
    return false;
  }
  const auto& config = std::get<fcconfig::block_sel::block_pickup_policy_config>(
      mc_matrix->find(bselm::kPickupPolicy)->second);

  /*
   * Unless we have the cluster proximity policy, we are good to go on
   * validation if we make it this far.
   */
  if (bselm::kPickupPolicyClusterProx == config.policy) {
    auto range = mc_map->values_range();
    if (!range.empty()) {
      auto avg_position =
          std::accumulate(range.begin(),
                          range.end(),
                          rmath::vector2d(),
                          [&](rmath::vector2d& sum, const auto& bent) {
                            return sum + bent.ent()->ranchor2D();
                          }) /
          boost::size(range);

      return (loc - avg_position).length() < config.prox_dist;
    }
  }
  return true;
} /* operator()() */

NS_END(fsm, fordyca);
