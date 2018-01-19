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
#include "rcppsw/ds/grid2D_ptr.hpp"
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, representation);
class block;
class cache;
class cell2D;

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
class line_of_sight {
 public:
  line_of_sight(const rcppsw::ds::grid_view<cell2D*>& c_view,
                discrete_coord center)
      : m_center(std::move(center)), m_view(c_view), m_caches() {}

  std::list<const block*> blocks(void) const;
  std::list<const cache*> caches(void) const;

  /**
   * @brief Add a cache to the LOS, beyond those that currently exist in the
   * LOS.
   *
   * This function is sometimes necessary because there unless the cell that a
   * cache actually resides in falls within a robot's LOS, they will not
   * actually see the cache, even if part of the cache's extent overlaps with
   * the LOS. This can cause robots to get a \ref cached_block_pickup event from
   * a cache that they do not currently track in their \ref perceived_arena_map,
   * which is bad.
   */
  void cache_add(const cache* cache);

  /**
   * @brief Get the size of the X dimension for a LOS.
   *
   * @return The X dimension.
   */
  size_t sizex(void) const { return m_view.shape()[0]; }

  discrete_coord abs_ll(void) const;
  discrete_coord abs_lr(void) const;
  discrete_coord abs_ul(void) const;
  discrete_coord abs_ur(void) const;

  /**
   * @brief Get the size of the Y dimension for a LOS.
   *
   * @return The Y dimension.
   */
  size_t sizey(void) const { return m_view.shape()[1]; }

  /**
   * @brief Get the # elements in a LOS.
   *
   * @return # elements.
   */
  size_t size(void) const { return m_view.num_elements(); }

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
  cell2D& cell(size_t i, size_t j) const;

  /**
   * @brief Get the coordinates for the center of the LOS.
   *
   * @return The center coordinates (discrete version).
   */
  const discrete_coord& center(void) const { return m_center; }

 private:
  void add_caches_from_view(void);
  void add_blocks_from_view(void);

  // clang-format off
  discrete_coord                 m_center;
  rcppsw::ds::grid_view<cell2D*> m_view;
  std::list<const cache*>        m_caches;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_LINE_OF_SIGHT_HPP_ */
