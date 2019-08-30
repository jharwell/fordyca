/**
 * @file unicell_movable_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_UNICELL_MOVABLE_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_UNICELL_MOVABLE_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/repr/unicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class unicell_movable_entity
 * @ingroup fordyca repr
 *
 * @brief A class representing objects that reside within one or more squares
 * within a 2D grid whose position CAN change during the lifetime of the object.
 */
class unicell_movable_entity : public unicell_entity {
 public:
  using unicell_entity::dloc;
  using unicell_entity::rloc;
  using unicell_entity::unicell_entity;

  static constexpr bool kIsMovable = true;

  ~unicell_movable_entity(void) override = default;

  void rloc(const rmath::vector2d& loc) {
    unicell_entity::rloc<unicell_movable_entity>(loc);
  }
  void dloc(const rmath::vector2u& loc) {
    unicell_entity::dloc<unicell_movable_entity>(loc);
  }

  bool is_movable(void) const override { return false; }
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_UNICELL_MOVABLE_ENTITY_HPP_ */
