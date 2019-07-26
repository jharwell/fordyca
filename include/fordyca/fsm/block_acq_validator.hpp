/**
 * @file block_acq_validator.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_ACQ_VALIDATOR_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_ACQ_VALIDATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/goal_acq_metrics.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace ds {
class dp_block_map;
} /* namespace ds */
namespace controller {
class block_sel_matrix;
} /* namespace controller */

NS_START(fsm);
using acq_goal_type = metrics::fsm::goal_acq_metrics::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @lockclass _acq_validator
 * @ingroup fordyca fsm
 *
 * @brief Determine if the acquisition of a block at a specific location/with a
 * specific ID is currently valid, according to simulation parameters and
 * current simulation state.
 */
class block_acq_validator : public rer::client<block_acq_validator> {
 public:
  block_acq_validator(const ds::dp_block_map* map,
                      const controller::block_sel_matrix* matrix);

  block_acq_validator(const block_acq_validator& v) = delete;
  block_acq_validator& operator=(const block_acq_validator& v) = delete;

  bool operator()(const rmath::vector2d& loc, uint id) const RCSW_PURE;

 private:
  /* clang-format off */
  const ds::dp_block_map* const              mc_map;
  const controller::block_sel_matrix * const mc_matrix;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_ACQ_VALIDATOR_HPP_ */
