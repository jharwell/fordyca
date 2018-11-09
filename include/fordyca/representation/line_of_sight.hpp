/**
 * @file line_of_sight.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include <utility>
#include "rcppsw/ds/base_grid2D.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/dcoord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class cell2D;
}
NS_START(representation);
class base_block;
class base_cache;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class line_of_sight
 * @ingroup representation
 *
 * @brief A representation of the robot's current line-of-sight. The robot is
 * only able to update its internal state based on the information present in
 * the per-timestep updates to this object.
 *
 * The LOS for a robot is always square UNLESS the robot is near the edge of
 * the arena, and a square grid would result in out-of-bounds array accesses. In
 * that case, a truncated LOS is created.
 *
 * All coordinates within a LOS are relative to the LOS itself (not its location
 * within the arena). The origin is in the lower left corner of the LOS.
 */
class line_of_sight : public rcppsw::er::client<line_of_sight> {
 public:
  using block_list = std::list<std::shared_ptr<base_block>>;
  using const_block_list = std::list<std::shared_ptr<const base_block>>;
  using cache_list = std::list<std::shared_ptr<base_cache>>;
  using const_cache_list = std::list<std::shared_ptr<const base_cache>>;
  using grid_view = rcppsw::ds::grid_view<ds::cell2D>;

  line_of_sight(const grid_view& c_view, rcppsw::math::dcoord2 center)
      : ER_CLIENT_INIT("fordyca.representation.line_of_sight"),
        m_center(std::move(center)),
        m_view(c_view),
        m_caches() {}

  const_block_list blocks(void) const;
  const_cache_list caches(void) const;

  /**
   * @brief Get the size of the X dimension for a LOS.
   *
   * @return The X dimension.
   */
  size_t xsize(void) const { return m_view.shape()[0]; }

  rcppsw::math::dcoord2 abs_ll(void) const;
  rcppsw::math::dcoord2 abs_lr(void) const;
  rcppsw::math::dcoord2 abs_ul(void) const;
  rcppsw::math::dcoord2 abs_ur(void) const;

  /**
   * @brief Get the size of the Y dimension for a LOS.
   *
   * @return The Y dimension.
   */
  grid_view::size_type ysize(void) const { return m_view.shape()[1]; }

  /**
   * @brief Get the # elements in a LOS.
   *
   * @return # elements.
   */
  grid_view::size_type size(void) const { return m_view.num_elements(); }

  /**
   * @brief Get the cell associated with a particular grid location within the
   * LOS. Asserts that both coordinates are within the bounds of the grid
   * underlying the LOS.
   *
   * @param i The RELATIVE X coord within the LOS.
   * @param j The RELATIVE Y coord within the LOS.
   *
   * @return A reference to the cell.
   */
  const ds::cell2D& cell(uint i, uint j) const;
  ds::cell2D& cell(uint i, uint j);

  /**
   * @brief Get the coordinates for the center of the LOS.
   *
   * @return The center coordinates (discrete version).
   */
  const rcppsw::math::dcoord2& center(void) const { return m_center; }

 private:
  // clang-format off
  rcppsw::math::dcoord2             m_center;
  grid_view                         m_view;
  const_cache_list                  m_caches;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_ */
