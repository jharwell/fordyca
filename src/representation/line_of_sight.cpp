/**
 * @file line_of_sight.cpp
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

/*******************************************************************************
 * Includes
 *****************************************************************************/
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
line_of_sight::const_block_list line_of_sight::blocks(void) const {
  const_block_list blocks;
  for (size_t i = 0; i < m_view.shape()[0]; ++i) {
    for (size_t j = 0; j < m_view.shape()[1]; ++j) {
      cell2D* cell = m_view[i][j];
      assert(cell);
      if (cell->state_has_block()) {
        assert(cell->block());
        blocks.push_back(cell->block());
      }
    } /* for(j..) */
  }   /* for(i..) */
  return blocks;
} /* blocks() */

line_of_sight::const_cache_list line_of_sight::caches(void) const {
  const_cache_list caches = m_caches;

  for (size_t i = 0; i < m_view.shape()[0]; ++i) {
    for (size_t j = 0; j < m_view.shape()[1]; ++j) {
      cell2D* cell = m_view[i][j];
      assert(cell);
      if (cell->state_has_cache() || cell->state_in_cache_extent()) {
        auto cache = std::dynamic_pointer_cast<base_cache>(cell->entity());
        assert(nullptr != cache);
        assert(cache->n_blocks() >= base_cache::kMinBlocks);
        /*
         * We can't add the cache unconditionally, because cache host cells and
         * extent cells both refer to the same cache, and doing so will give you
         * double references to a single cache in a LOS, which can cause
         * problems with pheromone updating. See #433.
         */
        if (caches.end() == std::find(caches.begin(), caches.end(), cache)) {
          caches.push_back(cache);
        }
      }
    } /* for(j..) */
  }   /* for(i..) */

  return caches;
} /* caches() */

__rcsw_pure cell2D& line_of_sight::cell(size_t i, size_t j) const {
  assert(i < m_view.shape()[0]);
  assert(j < m_view.shape()[1]);
  return *m_view[i][j];
}

rcppsw::math::dcoord2 line_of_sight::abs_ll(void) const {
  return cell(0, 0).loc();
} /* abs_ll() */

rcppsw::math::dcoord2 line_of_sight::abs_ul(void) const {
  return cell(0, ysize() - 1).loc();
} /* abs_ul() */

rcppsw::math::dcoord2 line_of_sight::abs_lr(void) const {
  return cell(xsize() - 1, 0).loc();
} /* abs_lr() */

rcppsw::math::dcoord2 line_of_sight::abs_ur(void) const {
  return cell(xsize() - 1, ysize() - 1).loc();
} /* abs_ur() */

NS_END(representation, fordyca);
