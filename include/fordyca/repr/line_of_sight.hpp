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

#ifndef INCLUDE_FORDYCA_REPR_LINE_OF_SIGHT_HPP_
#define INCLUDE_FORDYCA_REPR_LINE_OF_SIGHT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include <utility>
#include "fordyca/ds/block_list.hpp"
#include "fordyca/ds/cache_list.hpp"
#include "rcppsw/ds/base_grid2D.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class cell2D;
}
namespace rmath = rcppsw::math;

NS_START(repr);
class base_block;
class base_cache;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class line_of_sight
 * @ingroup fordyca repr
 *
 * @brief A repr of the robot's current line-of-sight. The robot is
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
  using grid_view = rcppsw::ds::base_grid2D<ds::cell2D>::grid_view;
  using const_grid_view = rcppsw::ds::base_grid2D<ds::cell2D>::const_grid_view;

  line_of_sight(const const_grid_view& c_view, const rmath::vector2u& center)
      : ER_CLIENT_INIT("fordyca.repr.line_of_sight"),
        mc_center(center),
        mc_view(c_view) {}

  ds::block_list blocks(void) const;
  ds::cache_list caches(void) const;

  /**
   * @brief Get the size of the X dimension for a LOS.
   *
   * @return The X dimension.
   */
  size_t xsize(void) const { return mc_view.shape()[0]; }

  rmath::vector2u abs_ll(void) const;
  rmath::vector2u abs_lr(void) const;
  rmath::vector2u abs_ul(void) const;
  rmath::vector2u abs_ur(void) const;

  /**
   * @brief Get the size of the Y dimension for a LOS.
   *
   * @return The Y dimension.
   */
  grid_view::size_type ysize(void) const { return mc_view.shape()[1]; }

  /**
   * @brief Determine if the *ABSOLUTE* arena location is contained in the LOS.
   */
  bool contains_loc(const rmath::vector2u& loc) const;

  /**
   * @brief Get the # elements in a LOS.
   *
   * @return # elements.
   */
  grid_view::size_type size(void) const { return mc_view.num_elements(); }

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

  /**
   * @brief Get the coordinates for the center of the LOS.
   *
   * @return The center coordinates (discrete version).
   */
  const rmath::vector2u& center(void) const { return mc_center; }

 private:
  /* clang-format off */
  const rmath::vector2u mc_center;
  const const_grid_view mc_view;
  ds::cache_list        m_caches{};
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_LINE_OF_SIGHT_HPP_ */
