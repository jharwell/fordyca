/**
 * @file base_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class base_block;
class multicell_entity;
} // namespace representation

NS_START(support, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_distributor
 * @ingroup support block_dist
 *
 * @brief Base class for block distributors to enable use of strategy pattern.
 */
class base_distributor : public rcppsw::er::client {
 public:
  using block_vector = std::vector<std::shared_ptr<representation::base_block>>;
  using entity_list = std::list<const representation::multicell_entity*>;

  explicit base_distributor(std::shared_ptr<rcppsw::er::server> server)
      : client(server) {}
  virtual ~base_distributor(void) = default;

  /**
   * @brief Distribute a block in the specified area by trying each random
   * distributor in turn.
   *
   * @return \c TRUE if the block distribution was successful, \c FALSE
   * otherwise.
   */
  virtual bool distribute_block(std::shared_ptr<representation::base_block>& block,
                                entity_list& entities) = 0;

  /**
   * @brief Calls \ref distribute_block on each block.
   *
   * @return \c TRUE iff all block distributions were successful, \c FALSE
   * otherwise.
   */
  virtual bool distribute_blocks(block_vector& blocks, entity_list& entities) = 0;
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_BASE_DISTRIBUTOR_HPP_ */
