/**
 * @file block_data.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_BLOCK_DATA_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_BLOCK_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief This structure holds data about blocks by a robot, including
 * operations on that data as the result of events such as picking up/dropping
 * blocks.
 */
class block_data {
 public:
  block_data(void) :
      m_has_block(false), m_curr_block_idx(-1), m_cum_blocks(0) {}

  bool has_block(void) const { return m_has_block; }
  int block_idx(void) const { return m_curr_block_idx; }
  void reset(void) {
    m_has_block = false;
    m_curr_block_idx = -1;
    m_cum_blocks = 0;
  }
  void picked_up_block(int idx) {
    m_has_block = true;
    m_curr_block_idx = idx;
  }
  void dropped_in_nest(void) {
    m_has_block = false;
    m_curr_block_idx = -1;
    ++m_cum_blocks;
  }

 private:
  bool   m_has_block;       /// True when the robot is carrying a block.
  int    m_curr_block_idx;  /// Index of the current block in global array.
  size_t m_cum_blocks;      /// Total # blocks carried by this robot so far.
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_BLOCK_DATA_HPP_ */
