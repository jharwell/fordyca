/**
 * @file base_cell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_BASE_CELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_BASE_CELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/utils/color.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_cell_entity
 * @ingroup repr
 *
 * @brief A base class from which objects that are able to occupy a cell (or
 * cells) within a 2D grid derive.
 */
class base_cell_entity {
 public:
  /**
   * @brief Initialize an entity with a color.
   */
  explicit base_cell_entity(const rcppsw::utils::color& color)
      : base_cell_entity{color, -1} {}

  /**
   * @brief Initialize an entity with a color and a unique identifier.
   */
  base_cell_entity(const rcppsw::utils::color& color, int id)
      : m_id(id), m_display_id(false), m_color(color) {}

  base_cell_entity(const base_cell_entity& other) = default;
  base_cell_entity& operator=(const base_cell_entity& other) = default;

  virtual ~base_cell_entity(void) = default;

  /**
   * @brief Set if the ID of the object be displayed during simulation for
   * visualization purposes.
   */
  void display_id(bool display_id) { m_display_id = display_id; }

  /**
   * @brief Should the ID of the object be displayed during simulation for
   * visualization purposes?
   */
  bool display_id(void) const { return m_display_id; }

  /**
   * @brief Set the ID of the object.
   */
  void id(int id) { m_id = id; }

  /**
   * @brief Get the ID of the object.
   */
  int id(void) const { return m_id; }

  /**
   * @brief Get the color of the entity.
   */
  const rcppsw::utils::color& color(void) const { return m_color; }

 private:
  /* clang-format off */
  int                   m_id;
  bool                  m_display_id;
  rcppsw::utils::color  m_color;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_BASE_CELL_ENTITY_HPP_ */
