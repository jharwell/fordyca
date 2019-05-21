/**
 * @file block_acquisition_validator.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_ACQUISITION_VALIDATOR_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_ACQUISITION_VALIDATOR_HPP_

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
class dp_block_map;
} /* namespace ds */

NS_START(fsm);
using acq_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_acquisition_validator
    : public rer::client<block_acquisition_validator> {
 public:
  explicit block_acquisition_validator(const ds::dp_block_map* map);

  block_acquisition_validator(const block_acquisition_validator& v) = delete;
  block_acquisition_validator& operator=(const block_acquisition_validator& v) =
      delete;

  bool operator()(const rmath::vector2d& loc, uint id) const;

 private:
  /* clang-format off */
  const ds::dp_block_map* const mc_map;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_ACQUISITION_VALIDATOR_HPP_ */
