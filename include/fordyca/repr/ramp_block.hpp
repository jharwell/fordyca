/**
 * @file ramp_block.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_REPR_RAMP_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPR_RAMP_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class ramp_block
 * @ingroup fordyca repr
 *
 * @brief A repr of a ramp block within the arena. Ramp blocks are 2x1
 * cells in size. Ramped blocks only need X,Y dimensions, because they are only
 * handled concretely in the arena in 2D (3D is only for visualization purposes,
 * and I can cheat a bit there).
 */
class ramp_block : public base_block {
 public:
  explicit ramp_block(const rmath::vector2d& dim)
      : base_block(dim, ut::color::kBLUE, -1) {}

  ramp_block(const rmath::vector2d& dim, int id)
      : base_block(dim, ut::color::kBLUE, id) {}

  transport_metrics::block_type type(void) const override {
    return transport_metrics::kRamp;
  }
  std::unique_ptr<base_block> clone(void) const override {
    auto tmp = rcppsw::make_unique<ramp_block>(dims(), id());
    tmp->discrete_loc(this->discrete_loc());
    tmp->real_loc(this->real_loc());
    tmp->reset_robot_id();
    return tmp;
  } /* clone() */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_RAMP_BLOCK_HPP_ */
