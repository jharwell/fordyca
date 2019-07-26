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

#ifndef INCLUDE_FORDYCA_REPR_CUBE_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPR_CUBE_BLOCK_HPP_

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
 * @class cube_block
 * @ingroup fordyca repr
 *
 * @brief A representation of a cube block within the arena. Cube blocks are 1
 * cell in size.
 */
class cube_block final : public base_block {
 public:
  explicit cube_block(const rmath::vector2d& dim)
      : base_block(dim, rutils::color::kBLACK, -1) {}

  cube_block(const rmath::vector2d& dim, int id) noexcept
      : base_block(dim, rutils::color::kBLACK, id) {}

  repr::block_type type(void) const override {
    return repr::block_type::ekCUBE;
  }
  std::unique_ptr<base_block> clone(void) const override {
    std::unique_ptr<base_block> tmp = std::make_unique<cube_block>(dims(), id());
    tmp->dloc(this->dloc());
    tmp->rloc(this->rloc());
    tmp->reset_robot_id();
    return tmp;
  } /* clone() */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_CUBE_BLOCK_HPP_ */
