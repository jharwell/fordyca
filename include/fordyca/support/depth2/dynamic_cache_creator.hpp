/**
 * @file dynamic_cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace er = rcppsw::er;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class dynamic_cache_creator : public depth1::cache_creator,
                              public er::client<dynamic_cache_creator> {
 public:
  dynamic_cache_creator(representation::arena_grid& grid,
                        double cache_size, double resolution, double min_dist);

  /**
   * @brief Scan the entire list of blocks currently in the arena, and create
   * caches from all blocks that are close enough together.
   *
   * @return The list of current caches.
   */
  cache_vector create_all(block_vector& blocks) override;

 private:
  argos::CVector2 calc_center(const block_list& blocks);

  double m_min_dist;
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_ */
