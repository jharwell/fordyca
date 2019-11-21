/**
 * \file base_entity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_REPR_BASE_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_BASE_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_entity
 * \ingroup fordyca repr
 *
 * \brief A base class from which all entities in the arena derive.
 */
class base_entity {
 public:
  /**
   * \brief Calculate the span in X of an entity given its location and
   * dimension in X.
   *
   * \return The span in X of the entity.
   */
  static rmath::ranged xspan(const rmath::vector2d& loc, double xdim) {
    return rmath::ranged(loc.x() - 0.5 * xdim, loc.x() + 0.5 * xdim);
  }

  /**
   * \brief Calculate the span in Y of an entity given its location and
   * dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  static rmath::ranged yspan(const rmath::vector2d& loc, double ydim) {
    return rmath::ranged(loc.y() - 0.5 * ydim, loc.y() + 0.5 * ydim);
  }

  base_entity(void) : base_entity{-1} {}
  explicit base_entity(int id) : m_id(id) {}

  base_entity(const base_entity& other) = default;
  base_entity& operator=(const base_entity& other) = default;

  virtual ~base_entity(void) = default;

  /**
   * \brief Set the ID of the object.
   */
  void id(int id) { m_id = id; }

  /**
   * \brief Get the ID of the object.
   */
  int id(void) const { return m_id; }

  void vis_id(bool b) { m_vis_id = b; }
  bool vis_id(void) const { return m_vis_id; }

  /**
   * \brief Calculate the span in X of an entity given its location and
   * dimension in X (objects track their own location and dimension).
   *
   * \return The span in X of the entity.
   */
  virtual rmath::ranged xspan(void) const = 0;

  /**
   * \brief Calculate the span in Y of an entity given its location and
   * dimension in Y.
   *
   * \return The span in Y of the entity.
   */
  virtual rmath::ranged yspan(void) const = 0;

  /**
   * \brief Get the size of the entity in the X direction in real coordinates.
   */
  virtual double xdimr(void) const = 0;

  /**
   * \brief Get the size of the entity in the Y direction in real coordinates.
   */
  virtual double ydimr(void) const = 0;

  virtual bool is_movable(void) const { return false; }

 private:
  /* clang-format off */
  bool m_vis_id{false};
  int  m_id;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_BASE_ENTITY_HPP_ */
