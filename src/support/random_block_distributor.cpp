/**
 * @file random_block_distributor.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/random_block_distributor.hpp"
#include <algorithm>

#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/multicell_entity.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/math/utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_block_distributor::random_block_distributor(
    std::shared_ptr<rcppsw::er::server> server,
    representation::arena_grid::view& grid,
    double resolution)
    : base_block_distributor(server),
      m_resolution(resolution),
      m_rng(argos::CRandom::CreateRNG("argos")),
      m_grid(grid) {
  insmod("random_block_dist", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool random_block_distributor::distribute_blocks(
    block_vector& blocks,
    entity_list& entities) {
  for (auto &b : blocks) {
    if (!distribute_block(b, entities)) {
      return false;
    }
    /*
     * Now that the block has been distributed, it is another entity that needs
     * to be distributed around.
     */
    entities.push_back(b.get());
  } /* for(&b..) */
  return true;
} /* distribute_blocks() */

bool random_block_distributor::distribute_block(
    std::shared_ptr<representation::block>& block,
    const entity_list& entities) {

  rcppsw::math::dcoord2 dcoord;
  representation::cell2D * cell = nullptr;

  dcoord = find_avail_coord(entities);
  cell = m_grid[dcoord.first][dcoord.second];

  /*
   * You can only distribute blocks to cells that do not currently have
   * anything in them.
   */
  ER_ASSERT(!cell->state_has_block(),
            "FATAL: Destination cell already contains block");
  ER_ASSERT(!cell->state_has_cache(),
            "FATAL: Destination cell already contains cache");

  events::free_block_drop op(client::server_ref(),
                             block,
                             dcoord,
                             m_resolution);
    cell->accept(op);
    return verify_block_dist(*block, cell);
} /* distribute_block() */

bool random_block_distributor::verify_block_dist(
    const representation::block& block,
    const representation::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(representation::block::kOutOfSightDLoc != block.discrete_loc(),
           "ERROR: Block%d discrete coordinates still out of sight after "
           "distribution",
           block.id());
  ER_CHECK(representation::block::kOutOfSightRLoc != block.real_loc(),
           "ERROR: Block%d real coordinates still out of sight after distribution",
           block.id());

  ER_NOM("Block%d: real_loc=(%f, %f) discrete_loc=(%zu, %zu) ptr=%p",
         block.id(),
         block.real_loc().GetX(),
         block.real_loc().GetY(),
         block.discrete_loc().first,
         block.discrete_loc().second,
         reinterpret_cast<void*>(cell->block().get()));
  return true;

error:
  return false;
} /* verify_block_dist() */


rcppsw::math::dcoord2 random_block_distributor::find_avail_coord(
    const entity_list& entities) {
  uint x, y;
  argos::CRange<uint> area_xrange(0, m_grid.shape()[0]);
  argos::CRange<uint> area_yrange(0, m_grid.shape()[1]);
  do {}
  while (std::any_of(entities.begin(),
                     entities.end(),
                     [&](const representation::multicell_entity* ent) {
                       x = m_rng->Uniform(area_xrange);
                       y = m_rng->Uniform(area_yrange);
                       auto movable = dynamic_cast<const representation::movable_cell_entity*>(ent);
                       if (nullptr != movable) {
                         return ent->xspan(movable->real_loc()).value_within(x) ||
                             ent->yspan(movable->real_loc()).value_within(y);
                       }
                       auto immovable = dynamic_cast<const representation::immovable_cell_entity*>(ent);
                       ER_ASSERT(nullptr != immovable,
                                 "FATAL: Cell entity is neither movable nor immovable");
                       return ent->xspan(immovable->real_loc()).value_within(x) ||
                           ent->yspan(immovable->real_loc()).value_within(y);
                     }));
  return rcppsw::math::dcoord2(x, y);
} /* find_avail_coord() */

NS_END(support, fordyca);
