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

#ifndef INCLUDE_FORDYCA_REPR_BASE_BLOCK_HPP_
#define INCLUDE_FORDYCA_REPR_BASE_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "rcppsw/types/timestep.hpp"

#include "fordyca/metrics/blocks/transport_metrics.hpp"
#include "fordyca/repr/colored_entity.hpp"
#include "fordyca/repr/unicell_movable_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class base_block
 * @ingroup fordyca repr
 *
 * @brief Base class for representing blocks (i.e. things that robots carry
 * within the arena). Blocks have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class base_block : public unicell_movable_entity,
                   public colored_entity,
                   public metrics::blocks::transport_metrics,
                   public rpprototype::clonable<base_block> {
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
  base_block(const rmath::vector2d& dim, const rutils::color& color)
      : unicell_movable_entity(dim, -1), colored_entity(color) {}

  /**
   * @param dim 2 element vector of the dimensions of the block.
   * @param color The color of the block.
   * @param id The id of the block.
   */
  base_block(const rmath::vector2d& dim, const rutils::color& color, int id)
      : unicell_movable_entity(dim, id), colored_entity(color) {}

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
  bool dloccmp(const base_block& other) const {
    return this->dloc() == other.dloc();
  }

  /* transport metrics */
  void reset_metrics(void) override final;
  uint total_transporters(void) const override { return m_transporters; }
  rtypes::timestep total_transport_time(void) const override RCSW_PURE;
  rtypes::timestep initial_wait_time(void) const override RCSW_PURE;

  /**
   * @brief Update a block's state given that it has been picked up by a robot:
   *
   * - Increment the # of carries this block has undergone on its way back
   *   to the nest.
   * - Set its reference to the robot that carries it.
   * - Move the block's position out of sight so that it is not discoverable by
   *   robots.
   */
  void robot_pickup_event(int robot_id) {
    ++m_transporters;
    m_robot_id = robot_id;
    move_out_of_sight();
  }

  /**
   * @brief Set the time that the base_block is picked up for the first time
   * after being distributed in the arena.
   *
   * @param time The current simulation time.
   */
  void first_pickup_time(const rtypes::timestep& t);

  /**
   * @brief Set the time that the base_block dropped in the nest.
   *
   * @param time The current simulation time.
   */
  void nest_drop_time(const rtypes::timestep& t) { m_nest_drop_time = t; }

  /**
   * @brief Set the time that the base_block was distributed in the arena.
   */
  void distribution_time(const rtypes::timestep& t) { m_dist_time = t; }

  /**
   * @brief Reset the the base_blocks carried/not carried state when it is not
   * carried by a robot anymore, but has not yet made it back to the nest
   * (i.e. dropped in a cache).
   */
  void reset_robot_id(void) { m_robot_id = -1; }

  /**
   * @brief Determine if the base_block is currently out of sight.
   *
   * This should only happen if the base_block is being carried by a robot.
   */
  bool is_out_of_sight(void) const {
    return kOutOfSightDLoc == dloc() || kOutOfSightRLoc == rloc();
  }
  /**
   * @brief Get the ID/index of the robot that is currently carrying this
   * block
   *
   * @return The robot index, or -1 if no robot is currently carrying this
   * block.
   */
  int robot_id(void) const { return m_robot_id; }
  void robot_id(int id) { m_robot_id = id; }

 protected:
  /**
   * @brief Provided to derived classes implementing \ref clone() so that they
   * can correctly clone block metadata/metrics.
   */
  void copy_metrics(const base_block& other) {
    this->m_transporters = other.m_transporters;
    this->m_first_pickup_time = other.m_first_pickup_time;
    this->m_first_pickup = other.m_first_pickup;
    this->m_dist_time = other.m_dist_time;
    this->m_nest_drop_time = other.m_nest_drop_time;
  }

 private:
  /**
   * @brief Change the block's location to sometnnnhing outside the visitable space
   * in the arena when it is being carried by robot.
   */
  void move_out_of_sight(void);

  /* clang-format off */
  int              m_robot_id{-1};
  uint             m_transporters{0};
  bool             m_first_pickup{false};
  rtypes::timestep m_first_pickup_time{0};
  rtypes::timestep m_dist_time{0};
  rtypes::timestep m_nest_drop_time{0};
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BASE_BLOCK_HPP_ */
