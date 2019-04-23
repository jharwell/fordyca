/**
 * @file cache_acquisition_validator.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_CACHE_ACQUISITION_VALIDATOR_HPP_
#define INCLUDE_FORDYCA_FSM_CACHE_ACQUISITION_VALIDATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class dp_cache_map;
} /* namespace ds */

namespace controller {
class cache_sel_matrix;
} /* namespace controller */

NS_START(fsm);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_acquisition_validator
 * @ingroup fordyca fsm
 *
 * @brief Determine if the acquisition of a cache at a specific location/with a
 * specific ID is currently valid, according to simulation parameters and
 * current simulation state.
 */
class cache_acquisition_validator
    : public rer::client<cache_acquisition_validator> {
 public:
  cache_acquisition_validator(const ds::dp_cache_map* map,
                              const controller::cache_sel_matrix* csel_matrix);

  cache_acquisition_validator(const cache_acquisition_validator& v) = delete;
  cache_acquisition_validator& operator=(const cache_acquisition_validator& v) =
      delete;

  bool operator()(const rmath::vector2d& loc, int id, uint timestep) const;

  /* clang-format off */
  const controller::cache_sel_matrix* const mc_csel_matrix;
  const ds::dp_cache_map*      const        mc_map;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_CACHE_ACQUISITION_VALIDATOR_HPP_ */
