/**
 * @file unicell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <type_traits>

#include "fordyca/nsalias.hpp"
#include "fordyca/repr/base_entity.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class unicell_entity
 * @ingroup fordyca repr
 *
 * @brief Representation of an entity in the arena that:
 *
 * - Resides in a single cell ("unicell")
 * - Has an extent that can span multiple cells, even though the entity itself
 *   is confined to a single cell. A useful, simplifying abstraction in dealing
 *   with the kinds of entities that robots interact with (blocks, caches,
 *   etc.), because the handshaking logic is much simpler.
 */
class unicell_entity : public base_entity {
 public:
  ~unicell_entity(void) override = default;

  /**
   * @brief Get the real location (center) of the object.
   */
  const rmath::vector2d& rloc(void) const { return m_rloc; }

  /**
   * @brief Get the discretized coordinates of the center of the object, which
   * can be used to index into an arena_map.
   *
   */
  const rmath::vector2u& dloc(void) const { return m_dloc; }

  rmath::ranged xspan(void) const override final {
    return base_entity::xspan(rloc(), xdimr());
  }
  rmath::ranged yspan(void) const override final {
    return base_entity::yspan(rloc(), ydimr());
  }
  double xdimr(void) const override final { return m_dim.x(); }
  double ydimr(void) const override final { return m_dim.y(); }

  /**
   * @brief Determine if a real-valued point lies within the extent of the
   * entity for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot is on top of an entity.
   *
   * @param point The point to check.
   *
   * @return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  const rmath::vector2d& dims(void) const { return m_dim; }

 protected:
  unicell_entity(const rmath::vector2d& dim,
                 const rmath::vector2d& loc,
                 rtypes::discretize_ratio resolution)
      : unicell_entity{dim, loc, resolution, -1} {}

  unicell_entity(const rmath::vector2d& dim,
                 const rmath::vector2d& loc,
                 rtypes::discretize_ratio resolution,
                 int id)
      : base_entity(id),
        m_dim(dim),
        m_rloc(loc),
        m_dloc(rmath::dvec2uvec(loc, resolution.v())) {}

  explicit unicell_entity(const rmath::vector2d& dim)
      : unicell_entity{dim, -1} {}

  unicell_entity(const rmath::vector2d& dim, int id)
      : base_entity(id), m_dim(dim), m_rloc(), m_dloc() {}

  template <typename T, typename = bool>
  struct entity_is_movable : std::false_type {};

  /**
   * @brief In order for a derived class to be considered movable and be able to
   * change its initial position, it needs to define an "kIsMovable" class
   * constant.
   */
  template <typename T>
  struct entity_is_movable<
      T,
      RCPPSW_SFINAE_TYPE(!std::is_same<decltype(T::kIsMovable), void>::value)>
      : std::true_type {};

  /**
   * @brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(entity_is_movable<T>::value)>
  void rloc(const rmath::vector2d& loc) {
    m_rloc = loc;
  }

  /**
   * @brief SFINAE to allow only derived classes that mark themselves as movable
   * to change the initial position of the entity.
   */
  template <typename T, RCPPSW_SFINAE_FUNC(entity_is_movable<T>::value)>
  void dloc(const rmath::vector2u& loc) {
    m_dloc = loc;
  }

 private:
  /* clang-format off */
  rmath::vector2d m_dim;
  rmath::vector2d m_rloc;
  rmath::vector2u m_dloc;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_UNICELL_ENTITY_HPP_ */
