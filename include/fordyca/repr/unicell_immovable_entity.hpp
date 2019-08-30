/**
 * @file unicell_immovable_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_UNICELL_IMMOVABLE_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_UNICELL_IMMOVABLE_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"

#include "fordyca/repr/unicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class unicell_immovable_entity
 * @ingroup fordyca repr
 *
 * @brief A class representing objects that reside within one or more squares
 * within a 2D grid whose position CANNOT change during the lifetime of the
 * object.
 */
class unicell_immovable_entity : public unicell_entity {
 public:
  unicell_immovable_entity(const rmath::vector2d& dim,
                           const rmath::vector2d& loc,
                           rtypes::discretize_ratio resolution)
      : unicell_entity(dim, loc, resolution, -1) {}

  unicell_immovable_entity(const rmath::vector2d& dim,
                           const rmath::vector2d& loc,
                           rtypes::discretize_ratio resolution,
                           int id)
      : unicell_entity(dim, loc, resolution, id) {}

  ~unicell_immovable_entity(void) override = default;
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_IMMOVABLE_UNICELL_ENTITY_HPP_ */
