/**
 * @file arena_map.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/representation/grid2D.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "rcppsw/common/er_server.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The arena map stores a logical representation of the state of the
 * arena. Basically, it combines a 2D grid with sets of objects that populate
 * the grid and move around as the state of the arena changes.
 */
class perceived_arena_map: public rcppsw::common::er_client {
 public:
  perceived_arena_map(const struct perceived_grid_params* params,
                      const std::shared_ptr<rcppsw::common::er_server>& server =
                      rcppsw::common::g_null_server);

  std::list<const block*> blocks(void);
  perceived_cell2D& access(size_t i, size_t j) { return m_grid.access(i, j); }
  void update_relevance(void);

  /* events */
  void event_block_pickup(block* block);
  void event_new_los(const line_of_sight* los);

 private:
  std::shared_ptr<rcppsw::common::er_server> m_server;
  grid2D<perceived_cell2D> m_grid;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_ */
