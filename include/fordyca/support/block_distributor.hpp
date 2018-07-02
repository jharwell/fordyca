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
#include "fordyca/representation/extent_model.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/dcoord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
class arena_grid;
} // namespace representation
namespace params {
struct block_distribution_params;
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
 * re-dstributes individual blocks every time they are dropped in the nest.
 */
class block_distributor : public rcppsw::er::client {
 public:
  static constexpr char kDIST_SINGLE_SRC[] = "single_source";
  static constexpr char kDIST_RANDOM[] = "random";
  static constexpr char kMODEL_SHAPE_RECT[] = "rectangle";
  static constexpr char kMODEL_ORIENTATION_HOR[] = "horizontal";

  block_distributor(std::shared_ptr<rcppsw::er::server> server,
                    const struct params::block_distribution_params* params);

  block_distributor(const block_distributor& s) = delete;
  block_distributor& operator=(const block_distributor& s) = delete;

  /**
   * @brief Distribute a block in the arena.
   */
  void distribute_block(representation::arena_grid& grid,
                        std::shared_ptr<representation::block>& block);

  /**
   * @brief Get the range in X over which the block will be distributed during
   * single-source distribution, after all paddings, etc. from configuration are
   * applied. This does not include any additional "off limits" areas due to the
   * presence of other blocks.
   *
   * @param block_xdim Size of block to distribute in X.
   */
  argos::CRange<double> single_src_xrange(double block_xdim);

  /**
   * @brief Get the range in Y over which the block will be distributed during
   * single-source distribution, after all paddings, etc. from configuration are
   * applied. This does not include any additional "off limits" areas due to the
   * presence of other blocks.
   *
   * @param block_ydim Size of block to distribute in Y.
   */
  argos::CRange<double> single_src_yrange(double block_ydim);

 private:
  /**
   * @brief Distribute a block randomly in the arena (excluding nest extent)
   * during initialization or after a robot has brought it to the
   * nest. Collisions are not checked.
   *
   * @param block The block to place/distribute.
   */
  argos::CVector2 dist_random(std::shared_ptr<representation::block>& block);

  /**
   * @brief Distribute a block within a small range about 90% of the way between
   * the nest and the far wall. Assumes a horizontally rectangular arena.
   */
  argos::CVector2 dist_single_src(std::shared_ptr<representation::block>& block);

  /**
   * @brief Distribute a block within a [x_min, y_min], [x_max, y_max]
   * range. Only does the distribution--does not check for collisions.
   *
   * @return The distribution location.
   */
  argos::CVector2 dist_in_range(argos::CRange<double> x_range,
                                argos::CRange<double> y_range);
  argos::CVector2 dist_outside_range(double dimension,
                                     argos::CRange<double> x_range,
                                     argos::CRange<double> y_range);

  // clang-format off
  std::string                  m_dist_type;
  representation::extent_model m_arena_model;
  representation::extent_model m_nest_model;
  argos::CRandom::CRNG*        m_rng;
  // clang-format on
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DISTRIBUTOR_HPP_ */
