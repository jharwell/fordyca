/**
 * @file block_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_distributor {
 public:
  block_distributor(argos::CRange<argos::Real> arena_x,
                    argos::CRange<argos::Real> arena_y,
                    argos::CRange<argos::Real> nest_x,
                    argos::CRange<argos::Real> nest_y,
                    const struct block_params* params) :
      m_dist_model(params->dist_model),
      m_respawn(params->respawn),
      m_arena_x(arena_x),
      m_arena_y(arena_y),
      m_nest_x(nest_x),
      m_nest_y(nest_y),
      m_rng(argos::CRandom::CreateRNG("argos")) {}


  /**
   * @brief Distribute all blocks in the arena.
   */
  void distribute_blocks(std::vector<representation::block>& blocks,
                         bool first_time);
  void distribute_block(representation::block& block, bool first_time);

 private:
  /**
   * @brief Distribute a block randomly in the arena (excluding nest extent)
   * during initialization or after a robot has brought it to the
   * nest. Collisions are not checked, as having 2 blocks on 1 square is not a
   * big deal for now.
   *
   * @param i The index of the block to place/distribute.
   */
  void dist_random(representation::block& block);
  void dist_single_src(representation::block& block);
  argos::CVector2 dist_in_range(argos::CRange<argos::Real> x_range,
                                argos::CRange<argos::Real> y_range);
  argos::CVector2 dist_outside_range(argos::CRange<argos::Real> x_range,
                                     argos::CRange<argos::Real> y_range);

  block_distributor(const block_distributor& s) = delete;
  block_distributor& operator=(const block_distributor& s) = delete;

  std::string m_dist_model;
  bool m_respawn;
  argos::CRange<argos::Real> m_arena_x;
  argos::CRange<argos::Real> m_arena_y;
  argos::CRange<argos::Real> m_nest_x;
  argos::CRange<argos::Real> m_nest_y;
  argos::CRandom::CRNG* m_rng;
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTOR_HPP_ */
