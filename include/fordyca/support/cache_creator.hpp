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

#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"
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
                double min_dist, double cache_size) :
      er_client(server), m_min_dist(min_dist), m_cache_size(cache_size) {}

  std::vector<representation::cache> create_all(
      std::vector<representation::block>& blocks);

 private:
  representation::cache create_single(representation::cache::starter_pair_ref blocks);

  double m_min_dist;
  double m_cache_size;
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHE_CREATOR_HPP_ */
