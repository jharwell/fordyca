/**
 * @file cube_block.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CUBE_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CUBE_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class cube_block
 * @ingroup representation
 *
 * @brief A representation of a cube block within the arena. Cube blocks are 1
 * cell in size.
 */
class cube_block : public base_block {
 public:
  explicit cube_block(const rmath::vector2d& dim)
      : base_block(dim, ut::color::kBLACK, -1) {}

  cube_block(const rmath::vector2d& dim, int id)
      : base_block(dim, ut::color::kBLACK, id) {}

  transport_metrics::block_type type(void) const override {
    return transport_metrics::kCube;
  }
  std::unique_ptr<base_block> clone(void) const override {
    std::unique_ptr<base_block> tmp =
        rcppsw::make_unique<cube_block>(dims(), id());
    tmp->discrete_loc(this->discrete_loc());
    tmp->real_loc(this->real_loc());
    tmp->reset_robot_id();
    return tmp;
  } /* clone() */
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_CUBE_BLOCK_HPP_ */
