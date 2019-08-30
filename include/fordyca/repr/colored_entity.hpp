/**
 * @file colored_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_COLORED_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_COLORED_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class colored_entity
 * @ingroup fordyca repr
 *
 * @brief A mixin class representing cell entities that have a static color.
 */
class colored_entity {
 public:
  /**
   * @brief Initialize an entity with a color.
   */
  explicit colored_entity(const rcppsw::utils::color& color) : m_color(color) {}

  colored_entity(const colored_entity& other) = default;
  colored_entity& operator=(const colored_entity& other) = default;

  virtual ~colored_entity(void) = default;

  const rcppsw::utils::color& color(void) const { return m_color; }

 private:
  /* clang-format off */
  rcppsw::utils::color m_color;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_COLORED_ENTITY_HPP_ */
