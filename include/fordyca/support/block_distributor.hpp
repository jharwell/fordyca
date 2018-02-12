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
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <string>
#include "rcppsw/math/dcoord.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
}
namespace params {
struct block_params;
}

NS_START(support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_distributor
 * @ingroup support
 *
 * @brief Distributes all blocks as directed on simulation start, and then
 * re-dstributes individual blocks every time they are dropped in the nest
 * (unless respawn is not enabled).
 */
class block_distributor {
 public:
  block_distributor(argos::CRange<double> arena_x,
                    argos::CRange<double> arena_y,
                    argos::CRange<double> nest_x,
                    argos::CRange<double> nest_y,
                    const struct params::block_params* params);

  block_distributor(const block_distributor& s) = delete;
  block_distributor& operator=(const block_distributor& s) = delete;

  /**
   * @brief Distribute a block in the arena.
   */
  bool distribute_block(const representation::block& block,
                        argos::CVector2* coord);

  argos::CRange<double> single_src_xrange(void);

 private:
  /**
   * @brief Distribute a block randomly in the arena (excluding nest extent)
   * during initialization or after a robot has brought it to the
   * nest. Collisions are not checked, as having 2 blocks on 1 square is not a
   * big deal for now.
   *
   * @param block The block to place/distribute.
   */
  argos::CVector2 dist_random(const representation::block& block);

  /**
   * @brief Distribute a block within a small range about 90% of the way between
   * the nest and the far wall. Assumes a horizontally rectangular arena.
   */
  argos::CVector2 dist_single_src(const representation::block& block);

  argos::CVector2 dist_in_range(argos::CRange<double> x_range,
                                argos::CRange<double> y_range);
  argos::CVector2 dist_outside_range(double dimension,
                                     argos::CRange<double> x_range,
                                     argos::CRange<double> y_range);

  // clang-format off
  std::string           m_dist_model;
  argos::CRange<double> m_arena_x;
  argos::CRange<double> m_arena_y;
  argos::CRange<double> m_nest_x;
  argos::CRange<double> m_nest_y;
  argos::CRandom::CRNG* m_rng;
  // clang-format on
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTOR_HPP_ */
