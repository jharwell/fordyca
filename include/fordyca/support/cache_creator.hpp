/**
 * @file cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>

#include "fordyca/representation/real_coord.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/grid2D.hpp"
#include "fordyca/representation/cell2D.hpp"

#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_creator : public rcppsw::common::er_client {
 public:
  cache_creator(std::shared_ptr<rcppsw::common::er_server> server,
                representation::grid2D<representation::cell2D>& grid,
                std::vector<representation::block>& blocks,
                double min_dist, double cache_size);

  /**
   * @brief Scan the entire list of blocks currently in the arena, and create
   * caches from all blocks that are close enough together.
   *
   * @return The list of current caches.
   */
  std::vector<representation::cache> create_all(void);

 private:
  representation::cache create_single(std::list<representation::block*> blocks);
  argos::CVector2 calc_center(std::list<representation::block*> blocks);

  double m_min_dist;
  double m_cache_size;
  std::vector<representation::block>& m_blocks;
  representation::grid2D<representation::cell2D>& m_grid;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHE_CREATOR_HPP_ */
