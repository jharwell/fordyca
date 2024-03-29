/**
 * \file cache_acq_validator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/ds/cache_vector.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
} // namespace cosm::arena::repr

NS_START(fordyca);

namespace controller::cognitive {
class cache_sel_matrix;
} // namespace controller::cognitive

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_acq_validator
 * \ingroup fsm
 *
 * \brief Determine if the acquisition of a cache at a specific location/with a
 * specific ID is currently valid, according to simulation parameters and
 * current simulation state.
 */
class cache_acq_validator : public rer::client<cache_acq_validator> {
 public:
  cache_acq_validator(const fspds::dp_cache_map* dpo_map,
                      const controller::cognitive::cache_sel_matrix* csel_matrix,
                      bool for_pickup);

  cache_acq_validator(const cads::bcache_vectorno& caches,
                      const controller::cognitive::cache_sel_matrix* csel_matrix,
                      bool for_pickup);

  cache_acq_validator(const cache_acq_validator& v) = delete;
  cache_acq_validator& operator=(const cache_acq_validator& v) = delete;

  /**
   * \brief Determine if the robot's acquisition of a cache is valid, according
   * to parameters and the current state of simulation.
   */
  bool operator()(const rmath::vector2d& loc,
                  const rtypes::type_uuid& id,
                  const rtypes::timestep& t) const;

 private:
  bool pickup_policy_validate(const carepr::base_cache* cache,
                              const rtypes::timestep& t) const;

  /* clang-format off */
  const bool                                           mc_for_pickup;
  const controller::cognitive::cache_sel_matrix* const mc_csel_matrix;
  const cads::bcache_vectorno                          mc_caches;
  /* clang-format on */
};

NS_END(fsm, fordyca);
