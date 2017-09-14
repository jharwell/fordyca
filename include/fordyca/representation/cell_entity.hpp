/**
 * @file cell_entity.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief A base class from which objects that are able to occupy a cell within
 * a 2D grid derive.
 */
class cell_entity {
 public:
  cell_entity(double x_dim, double y_dim, argos::CColor color) :
      m_id(-1), m_display_id(false), m_x_dim(x_dim), m_y_dim(y_dim),
      m_color(color), m_real_loc(), m_discrete_loc() {}

  virtual ~cell_entity(void) {}

  /**
   * @brief Get the size of the cell entity in the X direction
   */
  double xsize(void) const { return m_x_dim; }

  /**
   * @brief Get the size of the cell entity in the y direction
   */
  double ysize(void) const { return m_y_dim; }

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
   * @brief Get the real location of the object.
   */
  const argos::CVector2& real_loc(void) const { return m_real_loc; }

  /**
   * @brief Get the discretized coordinates of the object, which can be used to
   * index into an arena_map.
   *
   */
  const discrete_coord& discrete_loc(void) const { return m_discrete_loc; }

  void real_loc(const argos::CVector2& loc) { m_real_loc = loc; }
  void discrete_loc(const discrete_coord& loc) { m_discrete_loc = loc; }

  /**
   * @brief Determine if a real-valued point lies within the extent of the entity
   * for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot is on top of an entity.
   *
   * @param point The point to check.
   *
   * @return TRUE if the condition is met, and FALSE otherwise.
   */
  bool contains_point(const argos::CVector2& point);

  /**
   * @brief Set the ID of the object.
   */
  void id(int id) { m_id = id; }

  /**
   * @brief Get the ID of the object.
   */
  int id(void) const { return m_id; }

 private:
  int m_id;
  bool m_display_id;
  double m_x_dim;
  double m_y_dim;
  argos::CColor m_color;
  argos::CVector2 m_real_loc;
  discrete_coord m_discrete_loc;
};

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
typedef std::pair<const cell_entity*, double> perceived_entity;

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_CELL_ENTITY_HPP_ */
