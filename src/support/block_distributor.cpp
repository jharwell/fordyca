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
#include <assert.h>

#include "fordyca/params/block_distribution_params.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/arena_grid.hpp"
#include "fordyca/events/free_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_distributor::kMODEL_SHAPE_RECT[];
constexpr char block_distributor::kMODEL_ORIENTATION_HOR[];
constexpr char block_distributor::kDIST_RANDOM[];
constexpr char block_distributor::kDIST_SINGLE_SRC[];

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_distributor::block_distributor(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::block_distribution_params* params)
    : client(server),
      m_dist_type(params->dist_type),
      m_arena_model(params->arena_model),
      m_nest_model(params->nest_model),
      m_rng(argos::CRandom::CreateRNG("argos")) {
  insmod("block_distributor", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);

  /*
   * @todo For now, assume horizontal rectangle--this will change in the
   * future...eventually.
   */
  assert(m_arena_model.shape == kMODEL_SHAPE_RECT);
  assert(m_arena_model.orientation == kMODEL_ORIENTATION_HOR);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_distributor::distribute_block(representation::arena_grid& grid,
                                         std::shared_ptr<representation::block>& block) {
  argos::CVector2 r_coord;
  representation::cell2D* cell = nullptr;
  while(true) {
    if (m_dist_type == kDIST_RANDOM) {
      r_coord = dist_random(block);
    } else if (m_dist_type == kDIST_SINGLE_SRC) {
      r_coord = dist_single_src(block);
    } else {
      ER_FATAL_SENTINEL("FATAL: Bad block distribution type");
      break;
    }
    rcppsw::math::dcoord2 d_coord =
        math::rcoord_to_dcoord(r_coord, grid.resolution());
    cell = &grid.access(d_coord.first, d_coord.second);

    /*
     * You can only distribute blocks to cells that do not currently have
     * anything in them.
     */
    if (!cell->state_has_block() && !cell->state_has_cache()) {
      events::free_block_drop op(client::server_ref(),
                                 block,
                                 d_coord.first,
                                 d_coord.second,
                                 grid.resolution());
      cell->accept(op);
      break;
    }
  } /* while() */

  /* blocks should not be out of sight after distribution... */
  ER_ASSERT(representation::block::kOutOfSightDLoc != block->discrete_loc(),
            "FATAL: Block%d discrete coordinates still out of sight after "
            "distribution",
            block->id());
  ER_ASSERT(representation::block::kOutOfSightRLoc != block->real_loc(),
            "FATAL: Block%d real coordinates still out of sight after distribution",
            block->id());

  ER_NOM("Block%d: real_loc=(%f, %f) discrete_loc=(%zu, %zu) ptr=%p",
         block->id(),
         block->real_loc().GetX(),
         block->real_loc().GetY(),
         block->discrete_loc().first,
         block->discrete_loc().second,
         reinterpret_cast<void*>(cell->block().get()));
} /* distribute_block() */

argos::CVector2 block_distributor::dist_random(
    std::shared_ptr<representation::block>& block) {
  return dist_outside_range(block->xsize(), m_nest_model.x, m_nest_model.y);
} /* dist_random() */

__rcsw_pure argos::CRange<double> block_distributor::single_src_xrange(
    double block_xdim) {
  return argos::CRange<double>(m_arena_model.x.GetMax() * 0.9 - 4 * block_xdim,
                               m_arena_model.x.GetMax() * 0.9);
} /* single_src_xrange() */

__rcsw_pure argos::CRange<double> block_distributor::single_src_yrange(
    double block_ydim) {
  return argos::CRange<double>(
      std::max(4 * block_ydim, m_nest_model.y.GetMin() - 4 * block_ydim),
      std::min(m_arena_model.y.GetMax() - 4 * block_ydim,
               m_nest_model.y.GetMax() + 4 * block_ydim));
} /* single_src_yrange() */

argos::CVector2 block_distributor::dist_single_src(
    std::shared_ptr<representation::block>& block) {
  /*
   * Find the 90% point between the nest and the source along the X (horizontal)
   * direction, and put all the blocks around there.
   */
  return dist_in_range(single_src_xrange(block->xsize()),
                       single_src_yrange(block->ysize()));
} /* dist_single_src() */

argos::CVector2 block_distributor::dist_in_range(argos::CRange<double> x_range,
                                                 argos::CRange<double> y_range) {
  return argos::CVector2(m_rng->Uniform(x_range), m_rng->Uniform(y_range));
} /* dist_in_range() */

argos::CVector2 block_distributor::dist_outside_range(
    double dimension,
    argos::CRange<double> x_range,
    argos::CRange<double> y_range) {
  double x, y;
  x_range.Set(x_range.GetMin() - dimension, x_range.GetMax() + dimension);
  y_range.Set(y_range.GetMin() - dimension, y_range.GetMax() + dimension);
  do {
    x = m_rng->Uniform(
        argos::CRange<double>(m_arena_model.x.GetMin() + dimension,
                              m_arena_model.x.GetMax() - dimension));
    y = m_rng->Uniform(
        argos::CRange<double>(m_arena_model.y.GetMin() + dimension,
                              m_arena_model.y.GetMax() - dimension));
  } while (x_range.WithinMinBoundIncludedMaxBoundIncluded(x) ||
           y_range.WithinMinBoundIncludedMaxBoundIncluded(y));
  return argos::CVector2(x, y);
} /* dist_outside_range() */

NS_END(support, fordyca);
