/**
 * @file base_block.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_BASE_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_BASE_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/blocks/transport_metrics.hpp"
#include "fordyca/representation/movable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
namespace prototype = rcppsw::patterns::prototype;
namespace ut = rcppsw::utils;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class base_block
 * @ingroup representation
 *
 * @brief Base class for representing blocks (i.e. things that robots carry
 * within the arena). Blocks have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class base_block : public multicell_entity,
                   public movable_cell_entity,
                   public metrics::blocks::transport_metrics,
                   public rcppsw::patterns::visitor::visitable_any<base_block>,
                   public prototype::clonable<base_block> {
 public:
  /**
   * @brief Out of sight location base_blocks are moved to when a robot picks
   * them up, for visualization/rending purposes.
   */
  static const rmath::vector2u kOutOfSightDLoc;
  static const rmath::vector2d kOutOfSightRLoc;

  /**
   * @param dim 2 element vector of the dimensions of the block.
   * @param color The color of the block.
   *
   * Using this constructor, blocks are assigned the next available id, starting
   * from 0.
   */
  base_block(const rmath::vector2d& dim, const ut::color& color)
      : multicell_entity(dim, color, -1), movable_cell_entity() {}

  /**
   * @param dim 2 element vector of the dimensions of the block.
   * @param color The color of the block.
   * @param id The id of the block.
   */
  base_block(const rmath::vector2d& dim, const ut::color& color, int id)
      : multicell_entity(dim, color, id), movable_cell_entity() {}

  ~base_block(void) override = default;

  /**
   * @brief Disallow direct object comparisons, because we may want to compare
   * for equality in terms of IDs or object locations, and it is better to
   * require explicit comparisons for BOTH, rather than just one. It also makes
   * it unecessary to have to remember which type the comparison operator==()
   * does for this class.
   */
  bool operator==(const base_block& other) const = delete;

  /**
   * @brief Compare two \ref base_block objects for equality based on their ID.
   */
  bool idcmp(const base_block& other) const { return this->id() == other.id(); }

  /**
   * @brief Compare two \ref base_block objects for equality based on their
   * discrete location.
   */
  bool loccmp(const base_block& other) const {
    return this->discrete_loc() == other.discrete_loc();
  }

  /* transport metrics */
  void reset_metrics(void) override;
  uint total_transporters(void) const override { return m_transporters; }
  double total_transport_time(void) const override;
  double initial_wait_time(void) const override;

  /**
   * @brief Increment the # of carries this block has undergone on its way back
   * to the nest.
   */
  void add_transporter(int robot_id) {
    ++m_transporters;
    m_robot_id = robot_id;
  }

  /**
   * @brief Set the time that the base_block is picked up for the first time after
   * being distributed in the arena.
   *
   * @param time The current simulation time.
   */
  void first_pickup_time(double time);

  /**
   * @brief Set the time that the base_block dropped in the nest.
   *
   * @param time The current simulation time.
   */
  void nest_drop_time(double time) { m_nest_drop_time = time; }

  /**
   * @brief Set the time that the base_block was distributed in the arena.
   */
  void distribution_time(double dist_time) { m_dist_time = dist_time; }

  /**
   * @brief Reset the the base_blocks carried/not carried state when it is not
   * carried by a robot anymore, but has not yet made it back to the nest
   * (i.e. dropped in a cache).
   */
  void reset_robot_id(void) { m_robot_id = -1; }

  /**
   * @brief change the base_block's location to something outside the visitable
   * space in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void);

  /**
   * @brief Determine if the base_block is currently out of sight.
   *
   * This should only happen if the base_block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSightDLoc == discrete_loc() || kOutOfSightRLoc == real_loc();
  }
  /**
   * @brief Get the ID/index of the robot that is currently carrying this
   * block
   *
   * @return The robot index, or -1 if no robot is currently carrying this
   * block.
   */
  int robot_id(void) const { return m_robot_id; }

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
  bool contains_point(const rmath::vector2d& point) const {
    return xspan(real_loc()).contains(point.x()) &&
           yspan(real_loc()).contains(point.y());
  }

 private:
  // clang-format off
  int    m_robot_id{-1};
  uint   m_transporters{0};
  bool   m_first_pickup{false};
  double m_first_pickup_time{0.0};
  double m_dist_time{0.0};
  double m_nest_drop_time{0.0};
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BASE_BLOCK_HPP_ */
