/**
 * @file cell_cache_extent.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_CACHE_EXTENT_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_CACHE_EXTENT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/math/dcoord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class arena_map;
class perceived_arena_map;
class base_cache;
} // namespace representation

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell_cache_extent
 * @ingroup events
 *
 * @brief Created whenever a cell needs to go from some other state to being
 * part of a cache's extent (dub). All the blocks (and the cache itself) live in
 * a single cell, but the cells that the cache covers need to be in a special
 * state in order to avoid corner cases when picking up from/dropping in a cache
 * (See #432).
 */
class cell_cache_extent
    : public cell_op,
      public visitor::can_visit<representation::arena_map>,
      public visitor::can_visit<representation::perceived_arena_map> {
 public:
  cell_cache_extent(const rcppsw::math::dcoord2& coord,
                    const std::shared_ptr<representation::base_cache> cache);

  /* stateless foraging */
  void visit(representation::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(representation::arena_map& map) override;

  /* stateful foraging */
  void visit(representation::perceived_arena_map& map) override;

 private:
  // clang-format off
  std::shared_ptr<representation::base_cache> m_cache;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_CACHE_EXTENT_HPP_ */
