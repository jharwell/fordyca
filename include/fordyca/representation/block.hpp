/**
 * @file block.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <assert.h>
#include <utility>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/representation/cell_entity.hpp"
#include "fordyca/metrics/collectible_metrics/block_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block
 *
 * @brief A representation of a block within the arena map. Blocks do not have
 * state (other than if they are currently being carried by a robot). Blocks
 * have both real (where they actually live in the world) and discretized
 * locations (where they are mapped to within the arena map).
 */
class block : public cell_entity,
              public metrics::collectible_metrics::block_metrics,
              public rcppsw::patterns::visitor::visitable_any<block> {
 public:
  explicit block(double dimension) :
      cell_entity(dimension, dimension, argos::CColor::BLACK),
      m_robot_index(-1), m_carries(0) {}

  /* metrics */
  size_t n_carries(void) const override { return m_carries; }

  /**
   * @brief Increment the # of carries this block has undergone on its way back
   * to the nest.
   */
  void add_carry(void) { ++m_carries; }

  /**
   * @brief Reset the state of the block (i.e. not carried by a robot anymore).
   */
  void reset_index(void) { m_robot_index = -1; }

  void move_out_of_sight(void);
  void reset_carries(void) { m_carries = 0; }

  /**
   * @brief Get the ID/index of the robot that is currently carrying this block
   *
   * @return The robot index, or -1 if no robot is currently carrying this block.
   */
  int robot_index(void) const { return m_robot_index; }
  void robot_index(int robot_index) { m_robot_index = robot_index; }

 private:
  int    m_robot_index;
  size_t m_carries;
};

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
typedef std::pair<const block*, double> perceived_block;

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BLOCK_HPP_ */
