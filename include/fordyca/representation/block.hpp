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
#include "fordyca/representation/unicell_entity.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/math/dcoord.hpp"
#include "fordyca/representation/movable_cell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prototype = rcppsw::patterns::prototype;

NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class block
 * @ingroup representation
 *
 * @brief A representation of a block within the arena map. Blocks do not have
 * state (other than if they are currently being carried by a robot). Blocks
 * have both real (where they actually live in the world) and discretized
 * locations (where they are mapped to within the arena map).
 */
class block : public unicell_entity,
              public movable_cell_entity,
              public rcppsw::patterns::visitor::visitable_any<block>,
              public prototype::clonable<block> {
 public:
  /**
   * @brief Out of sight location blocks are moved to when a robot picks them
   * up, for visualization/rending purposes.
   */
  static rcppsw::math::dcoord2 kOutOfSightDLoc;
  static argos::CVector2 kOutOfSightRLoc;

  explicit block(double dimension)
      : unicell_entity(dimension, rcppsw::utils::color::kBLACK, -1),
        movable_cell_entity(),
        m_robot_index(-1),
        m_carries(0) {}

  block(double dimension, int id)
      : unicell_entity(dimension, rcppsw::utils::color::kBLACK, id),
        movable_cell_entity(),
        m_robot_index(-1),
        m_carries(0) {}

  __rcsw_pure bool operator==(const block& other) const {
    return (this->id() == other.id());
  }

  /* metrics */
  /**
   * @brief Reset the metrics (# carries) for the block after it is dropped in
   * the nest.
   */
  void reset_metrics(void) { m_carries = 0; }
  uint n_carries(void) const { return m_carries; }

  /**
   * @brief Increment the # of carries this block has undergone on its way back
   * to the nest.
   */
  void add_carry(void) { ++m_carries; }

  std::unique_ptr<block> clone(void) const override;

  /**
   * @brief Reset the state of the block (i.e. not carried by a robot anymore).
   */
  void reset_index(void) { m_robot_index = -1; }

  /**
   * @brief change the block's location to something outside the visitable space
   * in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void);

  /**
   * @brief Determine if the block is currently out of sight.
   *
   * This should only happen if the block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSightDLoc == discrete_loc() || kOutOfSightRLoc == real_loc();
  }
  /**
   * @brief Get the ID/index of the robot that is currently carrying this block
   *
   * @return The robot index, or -1 if no robot is currently carrying this
   * block.
   */
  int robot_index(void) const { return m_robot_index; }
  void robot_index(int robot_index) { m_robot_index = robot_index; }

  /**
   * @brief Determine if a real-valued point lies within the extent of the
   * entity for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot is on top of an entity.
   *
   * @param point The point to check.
   *
   * @return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const argos::CVector2& point) const {
    return xspan(real_loc()).value_within(point.GetX()) &&
        yspan(real_loc()).value_within(point.GetY());
  }

 private:
  // clang-format off
  int    m_robot_index;
  size_t m_carries;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BLOCK_HPP_ */
