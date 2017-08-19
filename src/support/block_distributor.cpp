/**
 * @file block_distributor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/block_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_distributor::distribute_blocks(
    std::vector<representation::block>& blocks,
    bool first_time) {
  for (size_t i = 0; i < blocks.size(); ++i) {
    distribute_block(blocks[i], first_time);
  } /* for(i..) */
} /* distribute_blocks() */

void block_distributor::distribute_block(representation::block& block,
                                         bool first_time) {
  if (!m_respawn && !first_time) {
    return;
  } else if (m_dist_model == "random") {
    dist_random(block);
  } else if (m_dist_model == "single_source") {
    dist_single_src(block);
  }
} /* distribute_block() */

void block_distributor::dist_random(representation::block& block) {
  block.set_real_loc(dist_outside_range(block.dimension(),
                                        m_nest_x, m_nest_y));
  block.set_discrete_loc(
      representation::block::discrete_coord(
          block.real_loc().GetX() / m_resolution - 1,
          block.real_loc().GetY() / m_resolution - 1));
} /* dist_random() */

void block_distributor::dist_single_src(representation::block& block) {
  /*
   * Find the 3/4 point between the nest and the source along the Y (horizontal)
   * direction, and put all the blocks around there.
   */
  argos::CRange<argos::Real> y_range = m_nest_y;
  argos::CRange<argos::Real> x_range = argos::CRange<argos::Real>(
      m_arena_x.GetMax() * 0.75 - 0.5,
      m_arena_x.GetMax() * 0.75);
  block.set_real_loc(dist_in_range(x_range, y_range));
  block.set_discrete_loc(
      representation::block::discrete_coord(
          std::max<int>(0, block.real_loc().GetX() / m_resolution - 1),
          std::max<int>(0, block.real_loc().GetY() / m_resolution - 1)));
} /* dist_single_src() */

argos::CVector2 block_distributor::dist_in_range(
    argos::CRange<argos::Real> x_range,
    argos::CRange<argos::Real> y_range) {
  return argos::CVector2(m_rng->Uniform(x_range),
                         m_rng->Uniform(y_range));
} /* dist_in_range() */

argos::CVector2 block_distributor::dist_outside_range(
    double dimension,
    argos::CRange<argos::Real> x_range,
    argos::CRange<argos::Real> y_range) {
  double x, y;
  do {
    x = m_rng->Uniform(
        argos::CRange<argos::Real>(m_arena_x.GetMin() + dimension,
                                   m_arena_x.GetMax() - dimension));
    y = m_rng->Uniform(
        argos::CRange<argos::Real>(m_arena_y.GetMin() + dimension,
                                   m_arena_y.GetMax() - dimension));
  } while (x_range.WithinMinBoundIncludedMaxBoundIncluded(x) &&
           y_range.WithinMinBoundIncludedMaxBoundIncluded(y));
  return argos::CVector2(x, y);
} /* dist_outside_range() */

NS_END(support, fordyca);
