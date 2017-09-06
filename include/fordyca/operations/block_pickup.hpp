/**
 * @file block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_OPERATIONS_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_OPERATIONS_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/operations/block_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class arena_map;
class block;
} /* namespace representation */

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_pickup : public block_op, public rcppsw::common::er_client {
 public:
  block_pickup(const std::shared_ptr<rcppsw::common::er_server>& server,
               representation::block* block, size_t robot_index);
  ~block_pickup(void) { rmmod(); }

  void visit(representation::arena_map& map);

  /**
   * @brief Handle the event of a robot picking up a block, making updates to
   * the arena map as necessary.
   *
   * @param map The robot's arena map.
   */
  void visit(representation::perceived_arena_map& map);
  void visit(representation::block& block);

 private:
  block_pickup(const block_pickup& op) = delete;
  block_pickup& operator=(const block_pickup& op) = delete;

  size_t m_robot_index;
  representation::block* m_block;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};

NS_END(operations, fordyca);

#endif /* INCLUDE_FORDYCA_OPERATIONS_BLOCK_PICKUP_HPP_ */
