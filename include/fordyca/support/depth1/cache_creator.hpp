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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <argos/core/utility/math/vector2.h>

#include "fordyca/representation/real_coord.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/occupancy_grid.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_creator : public rcppsw::common::er_client {
 public:
  cache_creator(std::shared_ptr<rcppsw::common::er_server> server,
                representation::occupancy_grid& grid,
                double cache_size, double resolution);

  /**
   * @brief Scan the entire vector of blocks currently in the arena, and create
   * caches from some/all blocks.
   *
   * @return A vector of created caches.
   */
  virtual std::vector<representation::cache> create_all(
      std::vector<representation::block>& blocks) = 0;

 protected:
  representation::occupancy_grid& grid(void) const { return m_grid; }
  rcppsw::common::er_server* server(void) const { return m_server.get(); }
  representation::cache create_single(std::list<representation::block*> blocks,
                                      const argos::CVector2& center);

 private:
  double m_cache_size;
  double m_resolution;
  representation::occupancy_grid& m_grid;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};
NS_END(support, fordyca, depth1);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_ */
