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
 * Constructors/Destructor
 ******************************************************************************/
block_distributor::block_distributor(
    argos::CRange<argos::Real> arena_x,
    argos::CRange<argos::Real> arena_y,
    argos::CRange<argos::Real> nest_x,
    argos::CRange<argos::Real> nest_y,
    const struct block_params& params,
    std::shared_ptr<std::vector<representation::block>> blocks) :
    m_arena_x(arena_x),
    m_arena_y(arena_y),
    m_nest_x(nest_x),
    m_nest_y(nest_y),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_params(params),
    m_blocks(blocks) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_distributor::distribute_blocks(bool first_time) {
  for (size_t i = 0; i < m_params.n_blocks; ++i) {
    distribute_block(i, first_time);
  } /* for(i..) */
} /* distribute_blocks() */

void block_distributor::distribute_block(size_t i, bool first_time) {
  if (!m_params.respawn && !first_time) {
    return;
  } else if (m_params.dist_model == "random") {
    dist_random(i);
  } else if (m_params.dist_model == "single_source") {
    dist_single_src(i);
  }
} /* distribute_block() */

void block_distributor::dist_random(size_t i) {
  m_blocks->at(i).set_loc(dist_outside_range(m_nest_x, m_nest_y));
} /* dist_random() */

void block_distributor::dist_single_src(size_t i) {
  /*
   * Find the 3/4 point between the nest and the source along the Y (horizontal)
   * direction, and put all the blocks around there.
   */
  argos::CRange<argos::Real> y_range = m_nest_y;
  argos::CRange<argos::Real> x_range = argos::CRange<argos::Real>(m_arena_x.GetMax() * 0.75 - 0.5,
                                                                  m_arena_x.GetMax() * 0.75);
  m_blocks->at(i).set_loc(dist_in_range(x_range, y_range));
} /* dist_single_src() */

argos::CVector2 block_distributor::dist_in_range(
    argos::CRange<argos::Real> x_range,
    argos::CRange<argos::Real> y_range) {
  return argos::CVector2(m_rng->Uniform(x_range),
                         m_rng->Uniform(y_range));
} /* dist_in_range() */

argos::CVector2 block_distributor::dist_outside_range(
    argos::CRange<argos::Real> x_range,
    argos::CRange<argos::Real> y_range) {
  double x, y;
  do {
    x = m_rng->Uniform(m_arena_x);
    y = m_rng->Uniform(m_arena_y);
  } while (x_range.WithinMinBoundIncludedMaxBoundIncluded(x) &&
           y_range.WithinMinBoundIncludedMaxBoundIncluded(y));
  return argos::CVector2(x, y);
} /* dist_outside_range() */

NS_END(support, fordyca);
